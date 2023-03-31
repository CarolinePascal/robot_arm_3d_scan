#include "robot_arm_3d_scan/RealSenseCamera.h"

#include "RealsenseToPCL.hpp"
#include <pcl/io/pcd_io.h>

//TODO set parameters values to default camera when no ROS parameter is given
RealSenseCamera::RealSenseCamera() : m_mutex(true)
{
    // Get the list of devices currently present on the system
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    size_t device_count = devices.size();

    double startTime = ros::WallTime::now().toSec();

    while(!device_count && ros::WallTime::now().toSec() - startTime < 10.0)
    {
        devices = ctx.query_devices();
        device_count = devices.size();
    }
    if (!device_count)
    {
        ROS_ERROR("Unable to find realsense camera !");
        throw std::runtime_error("NO REALSENSE CAMERA FOUND");
    }

    // Get the first connected device 
    m_device = devices.front();
    ROS_INFO("Found RealSense camera : %s",m_device.get_info(RS2_CAMERA_INFO_NAME));

    // Enter advanced mode
    if (m_device.is<rs400::advanced_mode>())
    {
        // Get the advanced mode functionality
        rs400::advanced_mode advanced_mode_dev = m_device.as<rs400::advanced_mode>();

        if(m_nodeHandle.getParam("realsense_json_file_path",m_presetPath))
        {
            ROS_INFO("Loading preset file : %s",m_presetPath.c_str());

            // Load and configure .json file to device
            std::ifstream t(m_presetPath);
            std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
            advanced_mode_dev.load_json(str);
        }

        if(m_nodeHandle.getParam("disparity_shift",m_disparityShift))
        {
            ROS_INFO("Setting up disparity shift : %d",m_disparityShift);

            // Disparity shift
            STDepthTableControl table = advanced_mode_dev.get_depth_table();
            table.disparityShift = m_disparityShift;
            advanced_mode_dev.set_depth_table(table);
        }
    }
    else
    {
        ROS_WARN("Could not access advanced mode for requested device !");
    }

    // Get the device depth sensor
    rs2::depth_sensor depth_sensor = m_device.first<rs2::depth_sensor>();

    if(m_nodeHandle.getParam("depth_units",m_depthUnits))
    {
        ROS_INFO("Setting up depth units : %f",m_depthUnits);

        // Depth units
        depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, m_depthUnits);
    }

    //TODO Proper integration ?
    bool tmp;
    if(m_nodeHandle.getParam("auto_exposure",tmp))
    {
        ROS_INFO("Setting up auto exposure : %d",(int)tmp);
        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, (int)tmp);
    }

    if(m_nodeHandle.getParam("filters",m_filtersNames))
    {
        // Filters
        for(std::vector<std::string>::iterator it = m_filtersNames.begin(); it!= m_filtersNames.end(); it++)
        {
            ROS_INFO("Setting up filter : %s",(*it).c_str());
            if(*it == "temporal")
            {
                rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.40);
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
                temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
                m_filters.push_back(temp_filter);
            }
            else if(*it == "threshold")
            {
                rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
                if(m_nodeHandle.getParam("threshold_filter",m_thresholdFilterParameters))
                {
                    thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, m_thresholdFilterParameters[0]);
                    thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, m_thresholdFilterParameters[1]);
                }
                else
                {
                    thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
                    thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 1.0);
                }
                m_filters.push_back(thr_filter);
            }
        }
    }

    // Start the streaming pipeline for both depth and RGB data
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    m_pipe.start(cfg);

    // Create publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/point_cloud",1);

    //Create parameters services
    m_disparityShiftService = m_nodeHandle.advertiseService("/disparity_shift",&RealSenseCamera::disparityShiftService,this);
    m_thresholdFilterService = m_nodeHandle.advertiseService("/threshold_filter",&RealSenseCamera::thresholdFilterService,this);
}

void RealSenseCamera::publishPointcloud()
{
    if(!m_mutex)
    {
        return;
    }

    // Wait for the next set of frames from the camera
    m_frames = m_pipe.wait_for_frames();
    m_depth = m_frames.get_depth_frame();
    m_color = m_frames.get_color_frame();

    // Apply filters
    for (int i = 0; i < m_filters.size(); i++)
    {
        m_depth = m_filters[i].process(m_depth);
        m_color = m_filters[i].process(m_color);
    }
    
    // Generate the pointcloud and texture mappings
    m_points = m_pc.calculate(m_depth);

    // Tell pointcloud object to map to this color frame
    m_pc.map_to(m_color);

    //Convert point cloud to ROS PCL msg
    m_pclPointCloud = PCL_Conversion(m_points, m_color);

    //Publish point cloud
    m_pclPointCloud->header.frame_id = "camera_link";
    pcl_conversions::toPCL(ros::Time::now(), m_pclPointCloud->header.stamp);
    m_pointCloudPublisher.publish(*m_pclPointCloud); 

    //DEBUG
    //pcl::io::savePCDFile(std::string("/tmp/PointCloud_pcd.pcd"), *m_pclPointCloud);
    //m_points.export_to_ply("/tmp/PointCloud_rs.ply",m_color);
}

bool RealSenseCamera::disparityShiftService(robot_arm_3d_scan::FloatParameters::Request& req, robot_arm_3d_scan::FloatParameters::Response& res)
{
    m_mutex = false;

    if (m_device.is<rs400::advanced_mode>())
    {
        m_disparityShift = req.parameters[0];

        // Get the advanced mode functionality
        rs400::advanced_mode advanced_mode_dev = m_device.as<rs400::advanced_mode>();

        ROS_INFO("Setting up disparity shift : %d",m_disparityShift);

        // Disparity shift
        STDepthTableControl table = advanced_mode_dev.get_depth_table();
        table.disparityShift = m_disparityShift;
        advanced_mode_dev.set_depth_table(table);

        m_mutex = true;
        return(true);
    }
    else
    {
        ROS_WARN("Could not access advanced mode for requested device !");

        m_mutex = true;
        return(false);
    }
}

bool RealSenseCamera::thresholdFilterService(robot_arm_3d_scan::FloatParameters::Request& req, robot_arm_3d_scan::FloatParameters::Response& res)
{
    m_mutex = false;

    for(int i = 0; i < m_filtersNames.size(); i++)
    {
        if(m_filtersNames[i] == "threshold")
        {
            ROS_INFO("Setting up filter : threshold");
            m_filters[i].set_option(RS2_OPTION_MIN_DISTANCE, req.parameters[0]);
            m_filters[i].set_option(RS2_OPTION_MAX_DISTANCE, req.parameters[1]);
        }
    }

    m_mutex = true;
    return(true);
}

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "realsense_camera_node");

    RealSenseCamera camera;

    while(ros::ok())
    {
        camera.publishPointcloud();
        ros::spinOnce();
    }  

    return 0;
}