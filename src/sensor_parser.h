#ifndef _SENSOR_PARSER_H_
#define _SENSOR_PARSER_H_

#define NUM_SENSOR_MIN 8
#define NUM_SENSOR_MAX 29
#define NUM_SENSORS 22

int n;// The number of bytes read
const int buffer_size = 100; // largest possible message size
unsigned char read_buf [buffer_size];
const int cb_size = buffer_size*12; // we want our circular buffer to be able to hold multiple messages
// Create a circular buffer with a capacity of cb_size
boost::circular_buffer<uint8_t> cb(cb_size);
bici_ros_sensor_reader::TactileData msg;

int setup_serialPort(void);
void parse_message(boost::circular_buffer<uint8_t> &cb,
					uint8_t start_index, uint8_t message_length,
					const std::array<ros::Publisher, NUM_SENSORS> &publishers);

#endif //_SENSOR_PARSER_H_
