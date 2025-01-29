/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <fg40_interfaces/msg/fg40_feedback.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/


/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 10;
static int const MKRCAN_MCP2515_INT_PIN = 2;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/
rcl_publisher_t publisher;
fg40_interfaces__msg__FG40Feedback msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void onReceiveBufferFull    (CanardFrame const &);
void onFeedback_0_1_Received(zubax::fluxgrip::Feedback_0_1 const & msg);

ArduinoMCP2515 mcp2515([]() { digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const data) { return SPI.transfer(data); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

static CanardPortID const INT_PORT_ID = 1000U;
cyphal::Publisher<uavcan::primitive::scalar::Integer8_1_0> integer_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>
  (INT_PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::Publisher<uavcan::node::Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);

static CanardPortID const FEEDBACK_PORT_ID   = 1001U;
cyphal::Subscription heartbeat_subscription = node_hdl.create_subscription<zubax::fluxgrip::Feedback_0_1>(FEEDBACK_PORT_ID, onFeedback_0_1_Received);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  delay(1000);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_1000kBPS_8MHZ);
  mcp2515.setNormalMode();

  /* Initialize ROS2 Node*/
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(fg40_interfaces, msg, FG40Feedback),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.magnetized = true;
  msg.remagnetization_state = 0;
  msg.cycles_on_off[0] = 1;
  msg.cycles_on_off[1] = 0;
}

void loop()
{

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_int = 0;
  unsigned long const now = millis();

  if(now - prev_heartbeat > 1000)
  {
    prev_heartbeat = now;

    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);
  }

  if(now - prev_int > 5000)
  {
    prev_int = now;

    uavcan::primitive::scalar::Integer8_1_0 msg;

    msg.value = 0;

    integer_pub->publish(msg);
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

void onFeedback_0_1_Received(zubax::fluxgrip::Feedback_0_1 const & msg)
{
  char msg_buf[64];
  snprintf(msg_buf, sizeof(msg_buf),
           "Magnetized = %s, Remag_State = %d, Cycles_on_off = %u",
           msg.magnetized ? "true" : "false", msg.remagnetization_state, msg.cycles_on_off);

  Serial.println(msg_buf);
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.magnetized = true;
    msg.remagnetization_state = 0;
    msg.cycles_on_off[0] = 1;
    msg.cycles_on_off[1] = 0;
  }
}