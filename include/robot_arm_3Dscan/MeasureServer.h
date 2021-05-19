/**
 * \file MeasureServer.h
 * \brief Header file of the MeasureServer abstract class
 *
 * Header file of the MeasureServer abstract class - Defines a generic interface for measurments ROS service servers
 *
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

 /*! \class MeasureServer
  * \brief Generic interface for measurments ROS service servers
  */
class MeasureServer
{
    public:
        /*!
         *  \brief Constructor
         */
        MeasureServer(std::string measureServerName);

        /*!
         *  \brief Destructor
         */
        virtual ~MeasureServer(){};

        /*!
         *  \brief Purely virtual method - Triggers the measurement
         */
        virtual bool measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) = 0;

    protected:

        ros::NodeHandle m_nodeHandle;   /*!< ROS node handle */
        std::string m_measureServerName;    /*!< ROS servce server name */
        ros::ServiceServer m_measureServer;    /*!< ROS service server used to trigger the measurement */
};