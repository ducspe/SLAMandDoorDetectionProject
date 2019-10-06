#include <iostream>
#include <vector>
#include <signal.h>
#include "sensor_device.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Range.h>
#include "tinkerforge_sensors_core.h"

using std::string;

void sigintHandler(int sig)
{
  ros::shutdown();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "tinkerforge_sensors");
  ros::NodeHandle n;

  // declare variables that can be modified by launch file or command line.
  int rate;
  int imu_convergence_speed;
  int port;
  string host;

  signal(SIGINT, sigintHandler);

  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("host", host, string("localhost"));
  private_node_handle_.param("port", port, int(4223)); 

  // create a new LaserTransformer object.
  TinkerforgeSensors *node_tfs = new TinkerforgeSensors(host, port);

  node_tfs->init();
  
  // read UIDs;
  std::vector<SensorConf> sensor_conf_vec;
  SensorConf sensor_conf;
  XmlRpc::XmlRpcValue list;


  if (n.getParam("/tfsensors/UID_to_Topic/", list))
  {
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = list.begin(); it != list.end(); ++it) {
      if (list[it->first].getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        //ROS_INFO_STREAM("Found filter: " << (std::string)(it->first) << " ==> " << list[it->first].getType());
        sensor_conf.uid = (std::string)(it->first);
        sensor_conf.topic = static_cast<std::string>(list[it->first]);
        sensor_conf_vec.push_back(sensor_conf);
      }
    }
    node_tfs->sensor_conf = sensor_conf_vec;
  }
  ros::Rate r(rate);

  // sleep a second for init sensors
  ros::Duration(1.0).sleep();

  // create publishers
  std::list<SensorDevice*>::iterator Iter;
  for (Iter = node_tfs->sensors.begin(); Iter != node_tfs->sensors.end(); ++Iter)
  {
    std::cout << "node::" << "::" << (*Iter)->getUID() << "::" << (*Iter)->getTopic() << std::endl;
    
    switch((*Iter)->getType())
    {
      case AMBIENT_LIGHT_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Illuminance>((*Iter)->getTopic().c_str(), 50));
      break;
      case AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Illuminance>((*Iter)->getTopic().c_str(), 50));
      case DISTANCE_IR_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Range>((*Iter)->getTopic().c_str(), 50));
      break;
      case DISTANCE_US_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Range>((*Iter)->getTopic().c_str(), 50));
      break;
      case GPS_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::NavSatFix>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Imu>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_V2_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Imu>((*Iter)->getTopic().c_str(), 50));
      break;
      case TEMPERATURE_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::Temperature>((*Iter)->getTopic().c_str(), 50));
      break;
      case IMU_V2_MAGNETIC_DEVICE_IDENTIFIER:
        (*Iter)->setPub(n.advertise<sensor_msgs::MagneticField>((*Iter)->getTopic().c_str(), 50));
      break;
    }
  }

  while (n.ok())
  {
    node_tfs->publishSensors();
    ros::spinOnce();
    r.sleep();
  }

  std::cout << "Shutdown node ...!" << std::endl;

  // clean up
  if (node_tfs != NULL)
    delete node_tfs;

  return 0;
}
