#include <Arduino.h>
#include <micro_ros_platformio.h>

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
void printf() {}
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA) || \
	defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
byte local_mac[] = LOCAL_MAC;
IPAddress local_ip(LOCAL_IP);
IPAddress agent_ip(AGENT_IP);
size_t agent_port = AGENT_PORT;
char ssid[] = WIFI_SSID;
char psk[] = WIFI_PASSWORD;
#endif

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
bool platformio_transport_open(struct uxrCustomTransport * transport) {return false;};
bool platformio_transport_close(struct uxrCustomTransport * transport) {return false;};
size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {return 0;};
size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {return 0;};
#endif

#ifndef ARGC_SETUP
#define ARGC_SETUP 0
#endif
#ifndef ARGV_SETUP
#define ARGV_SETUP NULL
#endif

int rmain(int argc, const char * const * argv);

void setup() {
	Serial.begin(921600);
#ifdef ESP32
    Serial.setRxBufferSize(1024);
#endif
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
	set_microros_serial_transports(Serial);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_NATIVE_ETHERNET)
	set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI) || defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI_NINA)
	set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_CUSTOM)
	rmw_uros_set_custom_transport(
		MICROROS_TRANSPORTS_FRAMING_MODE,
		NULL,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
#else
#error "No transport defined"
#endif
	rmain(ARGC_SETUP, ARGV_SETUP);
}

void loop() {
}

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>
#ifndef ARDUINO_DUE
#include <unistd.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		printf("Sent: %d\n", msg.data);
		msg.data++;
	}
}

int rmain(int argc, const char * const * argv)
{	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"std_msgs_msg_Int32"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;
	
  	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	return 0;
}