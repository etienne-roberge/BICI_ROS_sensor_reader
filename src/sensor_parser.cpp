/* This code receives an array of bytes from the USB serial port and stores it in a uint16 array that is published to a ros topic*/

#include <ros/ros.h>
#include <unistd.h> 
#include <stdio.h> 
#include <string.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h>
#include <iostream>
#include <bici_ros_sensor_reader/TactileData.h>
#include <termios.h>
#include <stdlib.h>
#include <boost/circular_buffer.hpp>
#include "sensor_parser.h"

#define PRINT
// #define TEST

int main (int argc, char** argv)
{
	ros::init(argc, argv, "serial_comm_node");
	ros::NodeHandle nh;

	/* Create Publishers */
	std::array<ros::Publisher, NUM_SENSORS> publishers;
	for (size_t i = 0; i < NUM_SENSORS; i++)
	{
		publishers[i] = nh.advertise<bici_ros_sensor_reader::TactileData>(
			"sensor_" + std::to_string(i + NUM_SENSOR_MIN) + "_readings", 1000);
	}

	/* Setup Serial Port */
	int serial_port = setup_serialPort();

	ros::Rate loop_rate(1000);

	/* Read and Parse Data */
	uint8_t byte_i; // the ith message of any given read
	uint8_t message_length; // the checksum we received in the message, encoding the message length
	int start_index = 0;
	int unread_bytes = 0;
	uint16_t end_index;

	while (ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();

		n = read(serial_port, &read_buf, sizeof(read_buf));

		if(n > 0)
		{
			// transfer data from read_buf into cb
			cb.insert(cb.end(), read_buf, read_buf + n);
			unread_bytes += n;
		}
		else if(unread_bytes <= 0)
		{
			continue; //Nothing to process, skip this loop
		}

		if (unread_bytes < cb_size)
		{
			bool msg_found = false;
			int start_index = 0;
			for (;start_index < unread_bytes && !msg_found; start_index++)
			{
				if (cb[start_index] == 1) //found start
				{
					// then we can get the message length
					int message_length = cb[start_index + 1];
					//is the end part of the message here?
					if (start_index + message_length > unread_bytes)
					{
						//std::cout << "its going to be part of the next message messLength=" << message_length << " unread=" << unread_bytes << std::endl;
						unread_bytes -= start_index;
						cb.erase_begin(start_index);
						msg_found = true;
					}
					else if (cb[start_index + message_length - 1] == '\n')
					{
						//traite le message
						parse_message(cb, start_index, message_length, publishers);
						unread_bytes -= start_index + message_length;
						std::cout << "Yeah valid message! messLength=" << message_length << " unread=" << unread_bytes << std::endl;
						msg_found = true;
						cb.erase_begin(start_index + message_length);
					}
				}
			}
			if(!msg_found)
			{
				cb.erase_begin(start_index);
				unread_bytes = 0;
				std::cout << "nothing good" << std::endl;
			}
		}
		else
		{
			std::cout << "ERROR: BUFFER OVERFLOW!!! Receiving data faster then we can process them." << std::endl;
			unread_bytes = 0;
		}


	}
}


int setup_serialPort(void)
{
	/* Open and Configure Serial Port */
	// open the serial port
	int serial_port = open("/dev/ttyUSB0", O_RDWR); //The serial port has to be properly renamed in certain cases

	// check for errors
	if (serial_port < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));
	}

	// create struct for configuring the serial port
	struct termios tty;

	// read in existing settings and check for errors errors
	if(tcgetattr(serial_port, &tty) != 0)
	{
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Setting other Port Stuff
	cfmakeraw(&tty);

	// configure VMIN and VTIME -- no blocking
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
	{
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		exit(1);
	}

	return serial_port;
}


void parse_message(boost::circular_buffer<uint8_t> &cb, uint8_t start_index, uint8_t message_length, const std::array<ros::Publisher, NUM_SENSORS> &publishers)
{
	// populate the timestamp field
	uint32_t timestamp;
	timestamp = (cb[start_index+3] << 24) | (cb[start_index+4] << 16) | (cb[start_index+5] << 8) | cb[start_index+6];
	msg.timestamp = timestamp;

	// push the data into the data field
	uint16_t data_entry;
	for (size_t i=7; i < message_length; i++)
	{
		// if i is odd
		if ((i % 2) != 0)
		{
			//grab MSB of data for current taxel
#ifndef TEST
			data_entry = (cb[(start_index + i) % cb_size]);
#endif

#ifdef TEST
			data_entry = (()[start_index + i] << 8);
#endif
		}

			// if i is even
		else
		{
			// grab LSB of data for current taxel
#ifndef TEST
			data_entry = data_entry | (cb[(start_index + i)%cb_size] << 8);
#endif

#ifdef TEST
			data_entry = data_entry | (cb[Z(start_index + i)]%cb_size);
#endif

			// push byte into msg.data
			msg.data.push_back(data_entry);
		}
	}

	// publish the message to the appropriate topic
	uint8_t sensor_num = cb[(start_index + 2)%cb_size];
	if ((sensor_num >= NUM_SENSOR_MIN) && (sensor_num <= NUM_SENSOR_MAX))
	{
		msg.sensor_num = sensor_num;
#ifdef PRINT
		//printf("sensor_num= %i, message_length = %i\n", sensor_num, message_length);
#endif
		publishers[sensor_num-NUM_SENSOR_MIN].publish(msg);
		msg.data.clear();
	}
	else
	{
#ifdef PRINT
		printf("BAD SENSOR NUM: start_byte = %i, checksum = %i, sensor_num=: %i \n", cb[start_index], cb[start_index+1], sensor_num);
#endif
	}

}