#include "robot_arm_3d_scan/RTABMAPServer.h"

RTABMAPServer::RTABMAPServer():MeasurementServer()
{
    m_pause = m_nodeHandle.serviceClient<std_srvs::Empty>("RTABMAP/pause");
    m_resume = m_nodeHandle.serviceClient<std_srvs::Empty>("RTABMAP/resume");

    m_pause.waitForExistence();

    std_srvs::Empty srv;
    if(!m_pause.call(srv))
    {
        throw std::runtime_error("FAILED TO REACH RTABMAP MEASURE SERVICE");
    }

    if(!m_nodeHandle.getParam("/rtabmap/rtabmap/Rtabmap/DetectionRate",m_acquisitionFrequency))
    {
        throw std::runtime_error("FAILED TO GET RTABMAP DETECTION RATE");
    }
}

bool RTABMAPServer::measure()
{
    std_srvs::Empty srv;

    if (!m_resume.call(srv))
    {
        throw std::runtime_error("FAILED TO RESUME RTABMAP MEASURE");
        return(false);
    }

    ros::WallDuration(1/m_acquisitionFrequency).sleep();

    if (!m_pause.call(srv))
    {
        throw std::runtime_error("FAILED TO PAUSE RTABMAP MEASURE");
        return(false);
    }

    return(true);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "rtabmap_server");  

    //RTABMAP measure service initialisation
    RTABMAPServer RTABMAPServer;

	ros::spin();
    return 0;
}