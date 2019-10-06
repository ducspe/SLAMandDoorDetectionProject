#ifndef SENSOR_DEVICE_H
#define SENSOR_DEVICE_H

#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include "ros/ros.h"
#include "bricklet_ambient_light.h"
#include "bricklet_ambient_light_v2.h"
#include "bricklet_distance_ir.h"
#include "bricklet_distance_us.h"
#include "bricklet_gps.h"
#include "brick_imu.h"
#include "brick_imu_v2.h"
#include "bricklet_temperature.h"

#define IMU_V2_MAGNETIC_DEVICE_IDENTIFIER 400

enum class SensorClass {TEMPERATURE, LIGHT, IMU, RANGE, GPS, MAGNETIC, MISC};

struct SensorConf
{
  std::string uid;
  std::string topic;
};

class SensorDevice
{
public:
  SensorDevice(void *dev, std::string uid, std::string topic, uint16_t type, SensorClass sclass, uint8_t rate)
  {
    this->dev = dev;
    this->uid = uid;
    this->seq = 0;
    this->type = type;
    this->sclass = sclass;
    this->rate = rate;
    this->frame = "imu_frame";//"/base_link";
    //this->pub = NULL;
    if (topic.size() == 0)
      buildTopic(this);
    else
      this->topic = topic;
  };
  ~SensorDevice()
  {
    std::cout << "Destructor for:" << uid << std::endl;
   };

   static std::string buildTopic(SensorDevice *sensor)
    {
      std::string topic;
      std::stringstream stream;

      dev_counter[(int)sensor->sclass]++;
      stream << SensorDevice::dev_counter[(int)sensor->sclass];
      switch (sensor->sclass)
      {
        case SensorClass::GPS:
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("gps") + std::string(stream.str());
        break;
        case SensorClass::IMU:
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("imu") + std::string(stream.str());
        break;
        case SensorClass::LIGHT:
          // conversion not working with my compiler
          // std::to_string(dev_counter[AMBIENT_LIGHT_DEVICE_IDENTIFIER])
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("illuminance") + std::string(stream.str());
        break;
        case SensorClass::MAGNETIC:
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("magnetic") + std::string(stream.str());
        break;
        case SensorClass::RANGE:
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("range") + std::string(stream.str());
        break;
        case SensorClass::TEMPERATURE:
          topic = std::string("/") + std::string("tfsensors") + std::string("/") + std::string("temperature") + std::string(stream.str());
        break;
      }
      sensor->topic = topic;
      return topic;
    };
public:
  void *getDev() { return dev; }
  std::string getUID() { return uid; }
  std::string getTopic() { return topic; }
  std::string getFrame() { return frame; }
  uint32_t getSeq() { seq++; return seq; }
  ros::Publisher getPub() { return pub; }
  uint16_t getType() { return type; }
  SensorClass getSensorClass() { return sclass; }

  void setTopic(std::string topic) { this->topic = topic; }
  void setPub(ros::Publisher pub) { this->pub = pub; }
  static int dev_counter[10];
  //void (*funcptr)(void*);
private:
  void *dev;
  std::string uid;
  std::string topic;
  std::string frame;
  uint32_t seq;
  uint16_t type;
  uint8_t rate;
  SensorClass sclass;
  ros::Publisher pub;
};
#endif
