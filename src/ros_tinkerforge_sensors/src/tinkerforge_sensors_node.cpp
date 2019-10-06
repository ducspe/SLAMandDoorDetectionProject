
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string.h>

#include "tinkerforge/ip_connection.h"
#include "tinkerforge/brick_imu_v2.h"

#define HOST "localhost"
#define PORT 4223

std::string frameId_ = "imu_frame";
ros::Publisher imuPub_;
const double deg2rad = M_PI/180.0;

double cov_orientation = 0.05;
double cov_velocity = 0.025;
double cov_acceleration = 0.1;

bool removeGravitationalAcceleration_ = true;

// Callback function for all data callback
void cb_all_data(int16_t acceleration[3], int16_t magnetic_field[3],
                 int16_t angular_velocity[3], int16_t euler_angle[3],
                 int16_t quaternion[4], int16_t linear_acceleration[3],
                 int16_t gravity_vector[3], int8_t temperature,
                 uint8_t calibration_status, void *user_data) {
	(void)user_data; // avoid unused parameter warning

	/*ROS_INFO("\nAcceleration        x: %.02f y: %.02f z: %.02f m/s²\n"
	       "Magnetic Field      x: %.02f y: %.02f z: %.02f µT\n"
	       "Angular Velocity    x: %.02f y: %.02f z: %.02f °/s\n"
	       "Euler Angle         x: %.02f y: %.02f z: %.02f °\n"
	       "Quaternion          x: %.02f y: %.02f z: %.02f w: %.02f\n"
	       "Linear Acceleration x: %.02f y: %.02f z: %.02f m/s²\n"
	       "Gravity Vector      x: %.02f y: %.02f z: %.02f m/s²\n"
	       "Temperature         %d °C\n"
	       "Calibration Status  %d\n\n",
	       acceleration[0]/100.0,        acceleration[1]/100.0,        acceleration[2]/100.0,
	       magnetic_field[0]/16.0,       magnetic_field[1]/16.0,       magnetic_field[2]/16.0,
	       angular_velocity[0]/16.0,     angular_velocity[1]/16.0,     angular_velocity[2]/16.0,
	       euler_angle[0]/16.0,          euler_angle[1]/16.0,          euler_angle[2]/16.0,
	       quaternion[1]/16383.0,        quaternion[2]/16383.0,        quaternion[3]/16383.0,        quaternion[0]/16383.0,
	       linear_acceleration[0]/100.0, linear_acceleration[1]/100.0, linear_acceleration[2]/100.0,
	       gravity_vector[0]/100.0,      gravity_vector[1]/100.0,      gravity_vector[2]/100.0,
	       temperature,
	       calibration_status);*/

	if(imuPub_.getNumSubscribers())
	{
		sensor_msgs::Imu imuMsg;
		imuMsg.header.frame_id = frameId_;
		imuMsg.header.stamp = ros::Time::now();

		boost::array<const double, 9> zeros =
						{ 0, 0, 0,
						  0, 0, 0,
						  0, 0, 0};

		imuMsg.orientation.w = quaternion[0]/16383.0;
		imuMsg.orientation.x = quaternion[1]/16383.0;
		imuMsg.orientation.y = quaternion[2]/16383.0;
		imuMsg.orientation.z = quaternion[3]/16383.0;

		imuMsg.orientation_covariance = zeros;
		imuMsg.orientation_covariance[0] = cov_orientation;
		imuMsg.orientation_covariance[4] = cov_orientation;
		imuMsg.orientation_covariance[8] = cov_orientation;

		imuMsg.angular_velocity.x = angular_velocity[0]/16.0*deg2rad;
		imuMsg.angular_velocity.y = angular_velocity[1]/16.0*deg2rad;
		imuMsg.angular_velocity.z = angular_velocity[2]/16.0*deg2rad;

		imuMsg.angular_velocity_covariance = zeros;
		imuMsg.angular_velocity_covariance[0] = cov_velocity;
		imuMsg.angular_velocity_covariance[4] = cov_velocity;
		imuMsg.angular_velocity_covariance[8] = cov_velocity;

		if(removeGravitationalAcceleration_)
		{
			imuMsg.linear_acceleration.x = linear_acceleration[0]/100.0;
			imuMsg.linear_acceleration.y = linear_acceleration[1]/100.0;
			imuMsg.linear_acceleration.z = linear_acceleration[2]/100.0;
		}
		else
		{
			imuMsg.linear_acceleration.x = acceleration[0]/100.0;
			imuMsg.linear_acceleration.y = acceleration[1]/100.0;
			imuMsg.linear_acceleration.z = acceleration[2]/100.0;
		}

		imuMsg.linear_acceleration_covariance = zeros;
		imuMsg.linear_acceleration_covariance[0] = cov_acceleration;
		imuMsg.linear_acceleration_covariance[4] = cov_acceleration;
		imuMsg.linear_acceleration_covariance[8] = cov_acceleration;

		//publish the message
		imuPub_.publish(imuMsg);
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "brick_imu");

	ros::NodeHandle pnh("~");
	pnh.param("frame_id", frameId_, frameId_);
	int period = 100;
	std::string uid = "6KwZJG"; // Change to your UID (shown by brick viewer)
	pnh.param("period_ms", period, period);
	pnh.param("uid", uid, uid);

	pnh.param("remove_gravitational_acceleration", removeGravitationalAcceleration_, removeGravitationalAcceleration_);

	pnh.param("cov_orientation", cov_orientation, cov_orientation);
	pnh.param("cov_velocity", cov_velocity, cov_velocity);
	pnh.param("cov_acceleration", cov_acceleration, cov_acceleration);

	ros::NodeHandle nh;
	imuPub_ = nh.advertise<sensor_msgs::Imu>("/tfsensors/imu1", 1);
	
	// Create IP connection
	IPConnection ipcon;
	ipcon_create(&ipcon);

	// Create device object
	IMUV2 imu;
	imu_v2_create(&imu, uid.c_str(), &ipcon);

	// Connect to brickd
	if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
		fprintf(stderr, "Could not connect\n");
		return 1;
	}
	// Don't use device before ipcon is connected

	// Register all data callback to function cb_all_data
	imu_v2_register_callback(&imu,
	                         IMU_V2_CALLBACK_ALL_DATA,
	                         (void *)cb_all_data,
	                         NULL);

	// Set period for all data callback to 0.1s (100ms)
	// Note: The all data callback is only called every 0.1 seconds
	//       if the all data has changed since the last call!
	imu_v2_set_all_data_period(&imu, period);

	//main loop
	ros::spin();

	ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally

	return 0;
}
