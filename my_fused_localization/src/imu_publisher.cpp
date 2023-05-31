#include "robot_localization/SetPose.h"

// C library headers
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int serial_port = open("/dev/ttyUSB0", O_RDWR);
std::string imuData;
ros::Publisher imu_pub;

//ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);
void serialCallback(const std_msgs::String::ConstPtr& msg)
{
    // Process the received string and fill the IMU message
    // Parse the received string and extract the relevant data
    // Populate the sensor_msgs/Imu message
    // Create a stringstream to process the string
     std::string input_string = msg->data;
    std::stringstream ss(input_string);

    // Create a vector to store the extracted numbers
    std::vector<double> numbers;

    std::string number;
    while (std::getline(ss, number, ',')) {
        // Convert the extracted string to a double and store it in the vector
        try {
            double num = std::stod(number);
            numbers.push_back(num);
        } catch (const std::invalid_argument& e) {
            // Handle the case where the conversion fails
            ROS_WARN("Invalid number format: %s", e.what());
        }
    }
    // Create a sensor_msgs/Imu message
    sensor_msgs::Imu imu_msg;
    // Verify that the vector contains the expected number of elements
    if (numbers.size() == 9) {
        // Assign the extracted numbers to the corresponding attributes of the Imu message
        imu_msg.linear_acceleration.x = numbers[0];
        imu_msg.linear_acceleration.y = numbers[1];
        imu_msg.linear_acceleration.z = numbers[2];

        imu_msg.orientation.x =  numbers[6];
        imu_msg.orientation.y =  numbers[7];
        imu_msg.orientation.z =  numbers[8];
        imu_msg.angular_velocity.x = numbers[3];
        imu_msg.angular_velocity.y = numbers[4];
        imu_msg.angular_velocity.z = numbers[5];

        // ... Populate other fields of the Imu message if needed
        imu_msg.header.frame_id = "base_link";
        imu_msg.header.stamp = ros::Time::now();
        for (size_t ind = 0; ind < 9; ind+=4)
        {
            imu_msg.angular_velocity_covariance[ind] = 1e-6;
            imu_msg.orientation_covariance[ind] = 1e-6;
        }
        imu_msg.linear_acceleration_covariance[0] = 1e-6;
        imu_msg.linear_acceleration_covariance[4] = 1e-6;
        imu_msg.linear_acceleration_covariance[8] = 1e-6;


        // Publish the filled IMU message
    } else {
        // Handle the case where the string does not contain the expected number of elements
        ROS_WARN("Invalid string format: Expected 6 numbers separated by commas");
    }
    // Fill the necessary fields of imu_msg based on the processed string data

    // Publish the filled IMU message
    ros::NodeHandle nh;

    imu_pub.publish(imu_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;
    // Create the publisher for the imu_data topic
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    // Create a subscriber to receive messages from serial_reader
    ros::Subscriber sub = nh.subscribe("serial_data", 10, serialCallback);

    ros::spin();

    return 0;
}
