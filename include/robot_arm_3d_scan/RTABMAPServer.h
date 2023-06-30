/**
 * \file RTABMAPServer.h
 * \brief Header file of the RTABMAPServer class
 *
 * Header file of the RTABMAPServer class - Defines the attributes and methods used to trigger RTABMAP measurements
 *
 */

#pragma once

#include "robot_arm_tools/MeasurementServer.h"

 /*! \class RTABMAPServer
  * \brief Class used to trigger RTABMAP measurements
  */
class RTABMAPServer : public MeasurementServer
{
    public:
        /*!
         *  \brief Constructor
         */
        RTABMAPServer();

        /*!
         *  \brief Destructor
         */
        ~RTABMAPServer(){};

        /*!
         *  \brief Triggers a RTABMAP measurement
         */
        bool measure();

    private:

        ros::ServiceClient m_pause,m_resume;    /*!< ROS service clients used to reach the RTABMAP node */
        double m_acquisitionFrequency;   /*!< RTABMAP acquisition frequency */
};