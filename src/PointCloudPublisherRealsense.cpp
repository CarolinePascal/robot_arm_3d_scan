#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp> // Include RealSense Advanced Cross Platform API

#include <algorithm>            // std::min, std::max
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "RealsenseToPCL.hpp"

#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "point_cloud_publisher");  
    ros::NodeHandle n;

    // Obtain a list of devices currently present on the system
    rs2::context ctx;
    auto devices = ctx.query_devices();
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
    auto dev = devices.front();

    // Enter advanced mode
    if (dev.is<rs400::advanced_mode>())
    {
        // Get the advanced mode functionality
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

        std::string presetPath;
        if(n.getParam("realsense_json_file_path",presetPath))
        {
            ROS_INFO("%s",presetPath.c_str());
            // Load and configure .json file to device
            std::ifstream t(presetPath);
            std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
            advanced_mode_dev.load_json(str);
        }

        int disparityShift;
        if(n.getParam("disparity_shift",disparityShift))
        {
            ROS_INFO("%d",disparityShift);
            // Disparity shift
            STDepthTableControl table = advanced_mode_dev.get_depth_table();
            table.disparityShift = disparityShift;
            advanced_mode_dev.set_depth_table(table);
        }
    }
    else
    {
        ROS_WARN("Could not access advanced mode for requested device !");
    }

    // Get the device depth sensor
    rs2::device selected_device = dev;
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    double depthUnits;
    if(n.getParam("depth_units",depthUnits))
    {
        ROS_INFO("%f",depthUnits);
        // Depth units
        depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, depthUnits);
    }

    std::vector<std::string> filtersNames;
    std::vector<rs2::filter> filters;
    if(n.getParam("filters",filtersNames))
    {
        // Filters
        for(std::vector<std::string>::iterator it = filtersNames.begin(); it!= filtersNames.end(); it++)
        {
            ROS_INFO("%s",(*it).c_str());
            if(*it == "temporal")
            {
                rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.40);
                temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
                temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
                filters.push_back(temp_filter);
            }
            else if(*it == "threshold")
            {
                rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
                thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.0);
                thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 0.5);
                filters.push_back(thr_filter);
            }
        }
    }

    // Start the streaming pipeline for both depth and RGB data
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    pipe.start(cfg);

    // Declare pointclouds
    rs2::frameset frames;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::frame color;
    rs2::frame depth;

    // Create PCL compatible point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcdPointCloud;

    // Create depth alignment object
    rs2::align align_to_depth(RS2_STREAM_DEPTH);

    // Create publisher !
    ros::Publisher pointCloudPublisher = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/point_cloud",1);

    while(ros::ok())
    {
        // Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames();
        //frames = align_to_depth.process(frames);
        depth = frames.get_depth_frame();
        color = frames.get_color_frame();

        // Apply filters
   	    for (int i = 0; i < filters.size(); i++)
        {
            depth = filters[i].process(depth);
            color = filters[i].process(color);
        }
        
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        //Convert point cloud to ROS PCL msg
        pcdPointCloud = PCL_Conversion(points, color);

        //Publish point cloud
        pcdPointCloud->header.frame_id = "camera_link";
        pcl_conversions::toPCL(ros::Time::now(), pcdPointCloud->header.stamp);
        pointCloudPublisher.publish(*pcdPointCloud);  
        ros::spinOnce();
    }

    //pcl::io::savePCDFile(std::string("/home/caroline/Bureau/robot_arm_3d_scan/config/scans/SphericScan_test/PointCloud_pcd.pcd"), *pcdPointCloud);
    //points.export_to_ply("/home/caroline/Bureau/robot_arm_3d_scan/config/scans/SphericScan_test/PointCloud_rs.ply",color);

    return(0);
}

