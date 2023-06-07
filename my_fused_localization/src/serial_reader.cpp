#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <serial/serial.h> // For serial communication

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_reader");
    ros::NodeHandle nh;

    // Set up the publisher to publish the received serial data
    ros::Publisher serial_pub = nh.advertise<std_msgs::String>("serial_data", 100);

    // Set up the serial port
    serial::Serial ser;
    ser.setPort("/dev/ttyACM0");  // Replace with the appropriate serial port
    ser.setBaudrate(9600);        // Set the baudrate to match your device

    try {
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Failed to open the serial port.");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port opened.");

        ros::Rate loop_rate(50);  // Adjust the rate according to your requirements

        while (ros::ok()) {
            if (ser.available()) {
                std_msgs::String msg;
                msg.data = ser.read(ser.available());
                serial_pub.publish(msg);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}
