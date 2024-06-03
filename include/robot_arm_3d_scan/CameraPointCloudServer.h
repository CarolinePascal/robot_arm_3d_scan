/**
 * \file CameraPointCloudServer.h
 * \brief Header file of the CameraPointCloudServer class
 *
 * Header file of the CameraPointCloudServer class - Defines the attributes and methods used to trigger point cloud measurements from a depth camera with dynamic parameters
 *
 */

#pragma once

#include "robot_arm_3d_scan/PointCloudServer.h"

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

 /*! \class CameraPointCloudServer
  * \brief Class used to trigger point cloud measurements from a depth camera with dynamic parameters
  */
class CameraPointCloudServer : public PointCloudServer
{
    public:
        /*!
         *  \brief Constructor
         */
        CameraPointCloudServer();

        /*!
         *  \brief Destructor
         */
        ~CameraPointCloudServer(){};

        /*!
         *  \brief Triggers a point cloud measurement from a depth camera with dynamic parameters
         */
        bool measure();

    private:

        std::vector<ros::ServiceClient> m_dynamicParametersClients; /*!< Vector of service clients to set dynamic parameters */
        std::vector<dynamic_reconfigure::Reconfigure> m_dynamicParametersSrvs; /*!< Vector of service requests to set dynamic parameters */
        int m_dynamicParametersNumber; /*!< Number of dynamic parameters */
};