#include "robot_arm_3Dscan/MeasureServer.h"

MeasureServer::MeasureServer(std::string measureServerName) : m_measureServerName(measureServerName)
{
    m_measureServer = m_nodeHandle.advertiseService(m_measureServerName,&MeasureServer::measure,this);
}