#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "ros/ros.h"
#include "sensor_device.h"
#include "tinkerforge_sensors_core.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Range.h>
#include "ip_connection.h"
#include "bricklet_ambient_light.h"
#include "bricklet_ambient_light_v2.h"
#include "brick_imu.h"
#include "brick_imu_v2.h"
#include "bricklet_gps.h"
#include "bricklet_industrial_digital_in_4.h"
#include "bricklet_dual_button.h"
#include "bricklet_temperature.h"
#include "bricklet_distance_ir.h"
#include "bricklet_distance_us.h"
#include "bricklet_motion_detector.h"
#include <tf/transform_broadcaster.h>

/*----------------------------------------------------------------------
 * TinkerforgeSensors()
 * Constructor
 *--------------------------------------------------------------------*/

TinkerforgeSensors::TinkerforgeSensors()
{
  imu_convergence_speed = 0;
}

TinkerforgeSensors::TinkerforgeSensors(std::string host, int port)
{
  if (host.length() == 0)
    this->host = "localhost";
  else
    this->host = host;
  if (port <= 1000)
    this->port = 4223;
  else
    this->port = port;
}

/*----------------------------------------------------------------------
 * ~TinkerforgeSensors()
 * Destructor
 *--------------------------------------------------------------------*/

TinkerforgeSensors::~TinkerforgeSensors()
{
  bool is_ipcon = false;
  // clean up tf devices
  while(!sensors.empty())
  {
    SensorDevice *dev = sensors.front();
    switch (dev->getType())
    {
      case AMBIENT_LIGHT_DEVICE_IDENTIFIER:
        ambient_light_destroy((AmbientLight*)dev->getDev());
      break;
      case AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER:
        ambient_light_v2_destroy((AmbientLightV2*)dev->getDev());
      break;
      case DISTANCE_IR_DEVICE_IDENTIFIER:
        distance_ir_destroy((DistanceIR*)dev->getDev());
      break;
      case DISTANCE_US_DEVICE_IDENTIFIER:
        distance_us_destroy((DistanceUS*)dev->getDev());
      break;
      case DUAL_BUTTON_DEVICE_IDENTIFIER:
        dual_button_destroy((DualButton*)dev->getDev());
      break;
      case GPS_DEVICE_IDENTIFIER:
        gps_destroy((GPS*)dev->getDev());
      break;
      case IMU_DEVICE_IDENTIFIER:
        imu_leds_off((IMU*)dev->getDev());
        imu_destroy((IMU*)dev->getDev());
      break;
      case IMU_V2_DEVICE_IDENTIFIER:
        imu_v2_leds_off((IMUV2*)dev->getDev());
        imu_v2_destroy((IMUV2*)dev->getDev());
      break;
      case TEMPERATURE_DEVICE_IDENTIFIER:
        temperature_destroy((Temperature*)dev->getDev());
      break;
    }
    delete dev;
    sensors.pop_front();
    is_ipcon = true;
  }
  if (is_ipcon)
  {
    ipcon_destroy(&ipcon);
  }
}

/*----------------------------------------------------------------------
 * Init()
 * Init the TF-Devices
 *--------------------------------------------------------------------*/

int TinkerforgeSensors::init()
{
  // create IP connection
  ipcon_create(&ipcon);

  // connect to brickd
  if(ipcon_connect(&ipcon, this->host.c_str(), this->port) < 0) {
    std::cout << "could not connect to brickd!" << std::endl;
    return false;
  }

  // register connected callback to "cb_connected"
  ipcon_register_callback(&ipcon,
    IPCON_CALLBACK_CONNECTED,
    (void*)callbackConnected,
    this);

  // register enumeration callback to "cb_enumerate"
  ipcon_register_callback(&ipcon,
    IPCON_CALLBACK_ENUMERATE,
    (void*)callbackEnumerate,
    this);

  return 0;
}

/*----------------------------------------------------------------------
 * publishImuMessage()
 * Publish the Imu message.
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::publishImuMessage(SensorDevice *sensor)
{
  int16_t acc_x, acc_y, acc_z;
  int16_t mag_x, mag_y, mag_z;
  int16_t ang_x, ang_y, ang_z;
  int16_t temp;
  float x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  int16_t ix = 0, iy = 0, iz = 0, iw = 0;
  ros::Time current_time = ros::Time::now();
  tf::TransformBroadcaster tf_broadcaster;
  if (sensor != NULL)
  {
    sensor_msgs::Imu imu_msg;

    // for the conversions look at rep 103 http://www.ros.org/reps/rep-0103.html
    // for IMU v1 http://www.tinkerforge.com/de/doc/Software/Bricks/IMU_Brick_C.html#imu-brick-c-api
    // for IMU v2 http://www.tinkerforge.com/de/doc/Software/Bricks/IMUV2_Brick_C.html#imu-v2-brick-c-api
    if (sensor->getType() == IMU_DEVICE_IDENTIFIER)
    {
      // check if imu is initialized
      //if (ros::Time::now().sec - imu_init_time.sec < 3)
      //  return;
      //else
      //  imu_set_convergence_speed(&imu, imu_convergence_speed);
      //
      imu_get_quaternion((IMU*)sensor->getDev(), &x, &y, &z, &w);

      imu_get_all_data((IMU*)sensor->getDev(), &acc_x, &acc_y, &acc_z, &mag_x, &mag_y,
        &mag_z, &ang_x, &ang_y, &ang_z, &temp);

      ang_x = ang_x / 14.375;
      ang_y = ang_y / 14.375;
      ang_z = ang_z / 14.375;

      acc_x = (acc_x/1000.0)*9.80605;
      acc_y = (acc_y/1000.0)*9.80605;
      acc_z = (acc_z/1000.0)*9.80605;
    }
    else if (sensor->getType() == IMU_V2_DEVICE_IDENTIFIER)
    {
      imu_v2_get_quaternion((IMUV2*)sensor->getDev(), &ix, &iy, &iz, &iw);
      x = ix / 16383.0;
      y = iy / 16383.0;
      z = iz / 16383.0;
      w = iw / 16383.0;

      imu_v2_get_linear_acceleration((IMUV2*)sensor->getDev(), &acc_x, &acc_y, &acc_z);
      imu_v2_get_angular_velocity((IMUV2*)sensor->getDev(), &ang_x, &ang_y, &ang_z);

      ang_x = ang_x * 16;
      ang_y = ang_y * 16;
      ang_z = ang_z * 16;

      acc_x = acc_x * 100;
      acc_y = acc_y * 100;
      acc_z = acc_z * 100;
    }
    else
    {
      return;
    }

    // message header
    imu_msg.header.seq = sensor->getSeq();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = sensor->getFrame();

    //TODO adapt values for IMU v2
    imu_msg.orientation.x = w;
    imu_msg.orientation.y = z*-1;
    imu_msg.orientation.z = y;
    imu_msg.orientation.w = x*-1;

    // orientation_covariance
    boost::array<const double, 9> oc =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};

    imu_msg.orientation_covariance = oc;

    // velocity from °/14.375 to rad/s
    imu_msg.angular_velocity.x = deg2rad(ang_x);
    imu_msg.angular_velocity.y = deg2rad(ang_y);
    imu_msg.angular_velocity.z = deg2rad(ang_z);

    // velocity_covariance
    boost::array<const double, 9> vc =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};
    imu_msg.angular_velocity_covariance = vc;

    // acceleration from mG to m/s²
    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;

    // linear_acceleration_covariance
    boost::array<const double, 9> lac =
      { 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1,
        0.1, 0.1, 0.1};
    imu_msg.linear_acceleration_covariance = lac;

    sensor->getPub().publish(imu_msg);
  }
}

/*----------------------------------------------------------------------
 * publishMagneticFieldMessage()
 * Publish the MagneticField message.
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::publishMagneticFieldMessage(SensorDevice *sensor)
{

  if (sensor != NULL)
  {
    if (sensor->getType() != IMU_V2_DEVICE_IDENTIFIER)
      return;

    int16_t x = 0, y = 0, z = 0;

    // for the conversions look at rep 103 http://www.ros.org/reps/rep-0103.html
    // for IMU v1 http://www.tinkerforge.com/de/doc/Software/Bricks/IMU_Brick_C.html#imu-brick-c-api
    // for IMU v2 http://www.tinkerforge.com/de/doc/Software/Bricks/IMUV2_Brick_C.html#imu-v2-brick-c-api
    if (sensor->getType() == IMU_DEVICE_IDENTIFIER)
    {
      imu_get_magnetic_field((IMU*)sensor->getDev(), &x, &y, &z); // nT -> T
      x = x / 10000000.0;
      y = y / 10000000.0;
      z = z / 10000000.0;
    }
    else
    {
      imu_v2_get_magnetic_field((IMUV2*)sensor->getDev(), &x, &y, &z); // µT -> T
      x = x / 1000000.0;
      y = y / 1000000.0;
      z = z / 1000000.0;
    }

    sensor_msgs::MagneticField mf_msg;

    // message header
    mf_msg.header.seq =  sensor->getSeq();
    mf_msg.header.stamp = ros::Time::now();
    mf_msg.header.frame_id = sensor->getFrame();

    // magnetic field from mG to T
    mf_msg.magnetic_field.x = x;
    mf_msg.magnetic_field.y = y;
    mf_msg.magnetic_field.z = z;

    boost::array<const double, 9> mfc =
      { 0.01, 0.01, 0.01,
        0.01, 0.01, 0.01,
        0.01, 0.01, 0.01};

    mf_msg.magnetic_field_covariance = mfc;

    sensor->getPub().publish(mf_msg);
  }
  return;
}

/*----------------------------------------------------------------------
 * publishNavSatFixMessage()
 * Publish the NavSatFix message.
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::publishNavSatFixMessage(SensorDevice *sensor)
{
  uint8_t fix, satellites_view, satellites_used;
  uint16_t pdop, hdop, vdop, epe;
  uint32_t latitude, longitude;
  uint32_t altitude, geoidal_separation;
  uint32_t course, speed;
  char ns, ew;
  if (sensor != NULL)
  {
    // get gps sensor status
    gps_get_status((GPS*)sensor->getDev(), &fix, &satellites_view, &satellites_used);

    if (fix != GPS_FIX_3D_FIX)
      return; // No valid data

    gps_get_coordinates((GPS*)sensor->getDev(), &latitude, &ns, &longitude, &ew, &pdop,
      &hdop, &vdop, &epe);
    gps_get_altitude((GPS*)sensor->getDev(), &altitude, &geoidal_separation);
    // course in deg, speed in 1/100 km/h
    gps_get_motion((GPS*)sensor->getDev(), &course, &speed);

    // generate NavSatFix message from gps sensor data
    sensor_msgs::NavSatFix gps_msg;

    // message header
    gps_msg.header.seq =  sensor->getSeq();
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = sensor->getFrame();
    // gps status
    gps_msg.status.status = gps_msg.status.STATUS_SBAS_FIX;
    gps_msg.status.service = gps_msg.status.SERVICE_GPS;

    gps_msg.latitude = latitude/1000000.0;
    gps_msg.longitude = longitude/1000000.0;
    gps_msg.altitude = altitude/100.0;
    gps_msg.position_covariance_type = gps_msg.COVARIANCE_TYPE_UNKNOWN;

    // publish gps msg to ros
    sensor->getPub().publish(gps_msg);
  }
}

/*----------------------------------------------------------------------
 * publishTemperatureMessage()
 * Publish the Temperature message.
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::publishTemperatureMessage(SensorDevice *sensor)
{
  if (sensor != NULL)
  {
    int16_t temperature;
    if(temperature_get_temperature((Temperature*)sensor->getDev(), &temperature) < 0) {
        ROS_ERROR_STREAM("Could not get temperature from " << sensor->getUID() << ", probably timeout");
        return;
    }
    // generate Temperature message from temperature sensor
    sensor_msgs::Temperature temp_msg;

    // message header
    temp_msg.header.seq =  sensor->getSeq();
    temp_msg.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = sensor->getFrame();

    temp_msg.temperature = temperature / 100.0;
    temp_msg.variance = 0;

    // publish Temperature msg to ros
    sensor->getPub().publish(temp_msg);
  }
}

/*----------------------------------------------------------------------
 * publishRangeMessage()
 * Publish the Range message.
 *--------------------------------------------------------------------*/
void TinkerforgeSensors::publishRangeMessage(SensorDevice *sensor)
{
  if (sensor != NULL)
  {
    // generate Range message from distance sensor
    sensor_msgs::Range range_msg;

    uint16_t distance;
    if (sensor->getType() == DISTANCE_US_DEVICE_IDENTIFIER)
    {
      if(distance_us_get_distance_value((DistanceUS*)sensor->getDev(), &distance) < 0) {
        ROS_ERROR_STREAM("Could not get range us from " << sensor->getUID() << ", probably timeout");
        return;
      }
      range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
      range_msg.range = distance;
      range_msg.field_of_view = 0.01;
      range_msg.min_range = 0.03;
      range_msg.max_range = 0.4;
    }
    else if (sensor->getType() == DISTANCE_IR_DEVICE_IDENTIFIER)
    {
      if(distance_ir_get_distance((DistanceIR*)sensor->getDev(), &distance) < 0) {
        ROS_ERROR_STREAM("Could not get range ir from " << sensor->getUID() << ", probably timeout");
        return;
      }
      range_msg.radiation_type = sensor_msgs::Range::INFRARED;
      range_msg.range = distance / 1000.0;
      range_msg.field_of_view = 0.01;
      range_msg.min_range = 0.03;
      range_msg.max_range = 0.4;
    }
    else
    {
      return;
    }

    // message header
    range_msg.header.seq =  sensor->getSeq();
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id = sensor->getFrame();

    // publish Range msg to ros
    sensor->getPub().publish(range_msg);
  }
}

/*----------------------------------------------------------------------
 * publishIlluminanceMessage()
 * Publish the Illuminance message.
 *--------------------------------------------------------------------*/
void TinkerforgeSensors::publishIlluminanceMessage(SensorDevice *sensor)
{
  if (sensor != NULL)
  {
    uint32_t illuminance = 0;

    // for the conversions look at rep 103 http://www.ros.org/reps/rep-0103.html
    // for Ambient Light v1 http://www.tinkerforge.com/de/doc/Software/Bricklets/AmbientLight_Bricklet_C.html
    // for Ambient Light v2 http://www.tinkerforge.com/de/doc/Software/Bricklets/AmbientLightV2_Bricklet_C.html

    if (sensor->getType() == AMBIENT_LIGHT_DEVICE_IDENTIFIER)
    {
      uint16_t ill = 0;
      // get current illuminance (unit is Lux/10)
      if(ambient_light_get_illuminance((AmbientLight*)sensor->getDev(), &ill) < 0) {
        ROS_ERROR_STREAM("Could not get illuminance from " << sensor->getUID() << ", probably timeout");
        return;
      }
      illuminance = (uint32_t)ill / 10.0;
    }
    else if (sensor->getType() == AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER)
    {
      // get current illuminance (unit is Lux/100)
      if (ambient_light_v2_get_illuminance((AmbientLightV2*)sensor->getDev(), &illuminance) < 0) {
        ROS_ERROR_STREAM("Could not get illuminance from " << sensor->getUID() << ", probably timeout");
        return;
      }
      illuminance = illuminance / 100.0;
    }
    else
    {
      return;
    }

    // generate Illuminance message from Ambient Light sensor
    sensor_msgs::Illuminance illum_msg;

    // message header
    illum_msg.header.seq =  sensor->getSeq();
    illum_msg.header.stamp = ros::Time::now();
    illum_msg.header.frame_id = sensor->getFrame();
    
    illum_msg.illuminance = illuminance;
    illum_msg.variance = 0;

    // publish Temperature msg to ros
    sensor->getPub().publish(illum_msg);
  }
}

/*----------------------------------------------------------------------
* publishSensors()
* Publish the Sensors messages.
*--------------------------------------------------------------------*/
void TinkerforgeSensors::publishSensors()
{
  std::list<SensorDevice*>::iterator lIter;
  for (lIter = sensors.begin(); lIter != sensors.end(); ++lIter)
  {
    //std::cout << "list::" << "::" << (*lIter)->getUID() << "::" << (*lIter)->getTopic() << std::endl;
    //if (lIter->getPub() == NULL)
    //  continue;
    switch((*lIter)->getSensorClass())
    {
      case SensorClass::LIGHT:
        publishIlluminanceMessage(*lIter);
      break;
      case SensorClass::IMU:
        publishImuMessage(*lIter);
      break;
      case SensorClass::MAGNETIC:
        publishMagneticFieldMessage(*lIter);
      break;
      case SensorClass::RANGE:
        publishRangeMessage(*lIter);
      break;
      case SensorClass::TEMPERATURE:
        publishTemperatureMessage(*lIter);
      break;
    }
  }

  for (std::vector<SensorConf>::iterator it = sensor_conf.begin(); it != sensor_conf.end(); ++it)
  {
    //std::cout << "vec::"<< it->uid << "::" << it->topic << std::endl;
  }
  return;
}

/*----------------------------------------------------------------------
 * callbackConnected()
 * Callback function for Tinkerforge ip connected
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::callbackConnected(uint8_t connect_reason, void *user_data)
{
  TinkerforgeSensors *tfs = (TinkerforgeSensors*) user_data;
  //if (tfs->is_imu_connected == false)
    ipcon_enumerate(&(tfs->ipcon));
  return;
}

/*----------------------------------------------------------------------
 * callbackEnumerate()
 * Callback function for Tinkerforge enumerate
 *--------------------------------------------------------------------*/

void TinkerforgeSensors::callbackEnumerate(const char *uid, const char *connected_uid,
                  char position, uint8_t hardware_version[3],
                  uint8_t firmware_version[3], uint16_t device_identifier,
                  uint8_t enumeration_type, void *user_data)
{
  TinkerforgeSensors *tfs = (TinkerforgeSensors*) user_data;
  std::string topic("");

  if(enumeration_type == IPCON_ENUMERATION_TYPE_DISCONNECTED)
  {
    return;
  }

  // search for predefined topic
  for (std::vector<SensorConf>::iterator it = tfs->sensor_conf.begin(); it != tfs->sensor_conf.end(); ++it)
  {
    if (it->uid.compare((std::string)uid) == 0)
    {
      topic = it->topic;
      break;
    }
  }

  // check if device is an imu
  if(device_identifier == IMU_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found IMU with UID:" << uid);
    // Create IMU device object
    IMU *imu = new IMU();
    imu_create(imu, uid, &(tfs->ipcon));
    imu_set_convergence_speed(imu,tfs->imu_convergence_speed);
    imu_leds_on(imu);
    tfs->imu_init_time = ros::Time::now();

    SensorDevice *imu_dev = new SensorDevice(imu, uid, topic, IMU_DEVICE_IDENTIFIER, SensorClass::IMU, 10);
    tfs->sensors.push_back(imu_dev);
  }
  else if (device_identifier == IMU_V2_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found IMU_v2 with UID:" << uid);
    // Create IMU_v2 device object
    IMUV2 *imu_v2 = new IMUV2();
    imu_v2_create(imu_v2, uid, &(tfs->ipcon));
    imu_leds_on(imu_v2);

    SensorDevice *imu_dev = new SensorDevice(imu_v2, uid, topic, IMU_V2_DEVICE_IDENTIFIER, SensorClass::IMU, 10);
    tfs->sensors.push_back(imu_dev);

    imu_dev = new SensorDevice(imu_v2, uid, std::string(""), IMU_V2_MAGNETIC_DEVICE_IDENTIFIER, SensorClass::MAGNETIC, 10);
    tfs->sensors.push_back(imu_dev);
    return;
  }
  else if (device_identifier == GPS_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found GPS with UID:" << uid);
    // Create GPS device object
    GPS *gps = new GPS();
    gps_create(gps, uid, &(tfs->ipcon));

    SensorDevice *gps_dev = new SensorDevice(gps, uid, topic, GPS_DEVICE_IDENTIFIER, SensorClass::GPS, 10);
    tfs->sensors.push_back(gps_dev);
  }
  else if (device_identifier == DUAL_BUTTON_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found DualButton with UID:" << uid);
    
    DualButton *db = new DualButton();
    dual_button_create(db, uid, &(tfs->ipcon));

    SensorDevice *db_dev = new SensorDevice(db, uid, topic, DUAL_BUTTON_DEVICE_IDENTIFIER, SensorClass::MISC, 10);
    tfs->sensors.push_back(db_dev);
  }
  else if (device_identifier == TEMPERATURE_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Temperature with UID:" << uid);
    Temperature *temp = new Temperature();
    // Create Temperature device object
    temperature_create(temp, uid, &(tfs->ipcon));

    SensorDevice *temp_dev = new SensorDevice(temp, uid, topic, TEMPERATURE_DEVICE_IDENTIFIER, SensorClass::TEMPERATURE, 10);
    tfs->sensors.push_back(temp_dev);

  }
  else if (device_identifier == AMBIENT_LIGHT_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Ambient Light with UID:" << uid);
    // Create Ambient Light device object
    AmbientLight *ambient_light = new AmbientLight();
    ambient_light_create(ambient_light, uid, &(tfs->ipcon));
    SensorDevice *ambient_light_dev = new SensorDevice(ambient_light, uid, topic, AMBIENT_LIGHT_DEVICE_IDENTIFIER, SensorClass::LIGHT, 10);
    tfs->sensors.push_back(ambient_light_dev);
  }
  else if (device_identifier == AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Ambient Light v2 with UID:" << uid);
    // Create Ambient Light device object
    AmbientLightV2 *ambient_v2_light = new AmbientLightV2();
    ambient_light_create(ambient_v2_light, uid, &(tfs->ipcon));
    SensorDevice *ambient_light_v2_dev = new SensorDevice(ambient_v2_light, uid, topic, AMBIENT_LIGHT_V2_DEVICE_IDENTIFIER, SensorClass::LIGHT, 10);
    tfs->sensors.push_back(ambient_light_v2_dev);
  }
  else if (device_identifier == DISTANCE_IR_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Distance IR with UID:" << uid);
    // Create Distance IR device object
    DistanceIR *distance_ir = new DistanceIR();
    distance_ir_create(distance_ir, uid, &(tfs->ipcon));
    SensorDevice *distance_ir_dev = new SensorDevice(distance_ir, uid, topic, DISTANCE_IR_DEVICE_IDENTIFIER, SensorClass::RANGE, 10);
    tfs->sensors.push_back(distance_ir_dev);
  }
  else if (device_identifier == DISTANCE_US_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Distance US with UID:" << uid);
    // Create Distance US  device object
    DistanceUS *distance_us = new DistanceUS();
    distance_us_create(distance_us, uid, &(tfs->ipcon));
    SensorDevice *distance_us_dev = new SensorDevice(distance_us, uid, topic, DISTANCE_US_DEVICE_IDENTIFIER, SensorClass::RANGE, 10);
    tfs->sensors.push_back(distance_us_dev);
  }
  else if (device_identifier == MOTION_DETECTOR_DEVICE_IDENTIFIER)
  {
    ROS_INFO_STREAM("found Motion Detector with UID:" << uid);
    // Create Motion Detector  device object
    MotionDetector * md = new MotionDetector();
    motion_detector_create(md, uid, &(tfs->ipcon));
    SensorDevice *md_dev = new SensorDevice(md, uid, topic, MOTION_DETECTOR_DEVICE_IDENTIFIER, SensorClass::MISC, 10);
    tfs->sensors.push_back(md_dev);
  }
}
