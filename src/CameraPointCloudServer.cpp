#include "robot_arm_3d_scan/CameraPointCloudServer.h"

//TODO Group parameters by "set_parameters" service, and combine parameters values in a single request
//TODO Retrieve default parameters values in constructor for recovery after measurement
//TODO Load parameters from Yaml ?


CameraPointCloudServer::CameraPointCloudServer() : PointCloudServer(), m_dynamicParametersNumber(0)
{
    //Get dynamic parameters
    XmlRpc::XmlRpcValue dynamicParameters;
    if(m_privateNodeHandle.getParam("dynamic_parameters",dynamicParameters))
    {
        for(int i = 0; i < dynamicParameters.size(); i++)
        {
            if(dynamicParameters[i].hasMember("name") && dynamicParameters[i].hasMember("type") && dynamicParameters[i].hasMember("value"))
            {
                ROS_INFO("Dynamic parameter %s of type %s with value %s",dynamicParameters[i]["name"].c_str(),dynamicParameters[i]["type"].c_str(),dynamicParameters[i]["value"].c_str());
            
                // Get service and parameter names
                std::string parameterName = dynamicParameters[i]["name"];
                std::size_t tmp = parameterName.find_last_of("/\\");
                std::string parameterService = parameterName.substr(0, tmp) + "/set_parameters";
                parameterName = parameterName.substr(tmp + 1);

                // Create client
                ros::ServiceClient client = m_nodeHandle.serviceClient<dynamic_reconfigure::Reconfigure>(parameterService);
                if(!client.waitForExistence(ros::Duration(5.0)))
                {
                    ROS_WARN("Could not find service %s",parameterName.c_str());
                    continue;
                }
                m_dynamicParametersClients.push_back(client);

                // Configure default request
                dynamic_reconfigure::Config config;
                std::string parameterType = dynamicParameters[i]["type"];
                if(parameterType == "int")
                {
                    int parameterValue = dynamicParameters[i]["value"];
                    dynamic_reconfigure::IntParameter param;
                    param.name = parameterName;
                    param.value = parameterValue;
                    config.ints.push_back(param);
                }
                elif(parameterType == "double")
                {
                    double parameterValue = dynamicParameters[i]["value"];
                    dynamic_reconfigure::DoubleParameter param;
                    param.name = parameterName;
                    param.value = parameterValue;
                    config.doubles.push_back(param);

                }
                elif(parameterType == "string")
                {
                    std::string parameterValue = dynamicParameters[i]["value"];
                    dynamic_reconfigure::StrParameter param;
                    param.name = parameterName;
                    param.value = parameterValue;
                    config.strs.push_back(param);
                }
                elif(parameterType == "bool")
                {
                    bool parameterValue = dynamicParameters[i]["value"];
                    dynamic_reconfigure::BoolParameter param;
                    param.name = parameterName; 
                    param.value = parameterValue;
                    config.bools.push_back(param);
                }
                else
                {
                    ROS_WARN("Unknown parameter type %s",parameterType.c_str());
                    continue;
                }

                dynamic_reconfigure::Reconfigure srv;
                srv.request.config = config;
                m_dynamicParametersSrvs.push_back(srv);
                m_dynamicParametersNumber++;
            }

        }
    }
}

bool CameraPointCloudServer::measure()
{
    for (int i = 0; i < m_dynamicParametersNumber; i++)
    {
        if (!m_dynamicParametersClients[i].call(m_dynamicParametersSrvs[i]))
        {
            ROS_WARN("Could not set dynamic parameter %d !", i);
            return false;
        }
        else
        {
            ros::WallDuration(1.0).sleep();
        }
    }

    bool output = PointCloudServer::measure();

    for (int i = 0; i < m_dynamicParametersNumber; i++)
    {
        if (!m_dynamicParametersClients[i].call(m_dynamicParametersSrvs[i]))
        {
            ROS_WARN("Could not set dynamic parameter %d !", i);
            return false;
        }
        else
        {
            ros::WallDuration(1.0).sleep();
        }
    }

    return output;
}