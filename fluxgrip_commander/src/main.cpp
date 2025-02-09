#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <fg40_interfaces/msg/fg40_magnet_cmd.h>
#include <fg40_interfaces/msg/fg40_feedback.h>

#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>
#include <107-Arduino-Cyphal-Support.h>

#include <queue>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
fg40_interfaces__msg__FG40MagnetCmd magnet_cmd_msg;
fg40_interfaces__msg__FG40Feedback feedback_msg;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

static int const MKRCAN_MCP2515_CS_PIN  = D17;
static int const MKRCAN_MCP2515_INT_PIN = D5;
static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

void onReceiveBufferFull    (CanardFrame const &);
void onFeedback_0_1_Received(zubax::fluxgrip::Feedback_0_1 const & recieved_fg40_msg);

ArduinoMCP2515 mcp2515([]()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

CanardPortID const CMD_PORT_ID = 1000U;
cyphal::Publisher<uavcan::primitive::scalar::Integer8_1_0> cyphal_cmd_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>
  (CMD_PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::Publisher<uavcan::node::Heartbeat_1_0> cyphal_heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);

static CanardPortID const FEEDBACK_PORT_ID   = 1001U;
cyphal::Subscription cyphal_feedback_sub;

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

void onFeedback_0_1_Received(zubax::fluxgrip::Feedback_0_1 const & recieved_fg40_msg)
{
  feedback_msg.magnetized = recieved_fg40_msg.magnetized;
  feedback_msg.remagnetization_state = recieved_fg40_msg.remagnetization_state;
  feedback_msg.cycles_on_off[0] = recieved_fg40_msg.cycles_on_off[0];
  feedback_msg.cycles_on_off[1] = recieved_fg40_msg.cycles_on_off[1];
}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &feedback_msg, NULL));
    // feedback_msg.magnetized = true;
    // feedback_msg.remagnetization_state = 0;
    // feedback_msg.cycles_on_off[0] = 1;
    // feedback_msg.cycles_on_off[1] = 0;
  }
}

//Listen to /fg40_cmd and publish cmd as a cyphal CAN message
void subscription_callback(const void * msgin)
{  
  const fg40_interfaces__msg__FG40MagnetCmd * msg = (const fg40_interfaces__msg__FG40MagnetCmd *)msgin;
  //digitalWrite(LED_BUILTIN, (msg->cmd_magnet == 1) ? LOW : HIGH);  

  int cmd = msg->cmd_magnet;
  uavcan::primitive::scalar::Integer8_1_0 cyphal_msg;

  switch(cmd){
    
    //Demagnetize FG40
    case 0:
    cyphal_msg.value = 0;
    cyphal_cmd_pub->publish(cyphal_msg);
    break;

    //Magnetize FG40
    case 1:
    cyphal_msg.value = 1;
    cyphal_cmd_pub->publish(cyphal_msg);
    break;

    //FORCE Magnetize/Demagnetize Cycle on FG40
    case 2:
    cyphal_msg.value = 2;
    cyphal_cmd_pub->publish(cyphal_msg);
    break;

    //Ignore other INT values and replace with a magnetize cmd
    default:
    cyphal_msg.value = 1;
    cyphal_cmd_pub->publish(cyphal_msg);
    break;

  }

}

void setup() {
    Serial.begin(115200);
    static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "MARS-X500.FG40-Commander"
  );

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_1000kBPS_8MHZ);
  mcp2515.setNormalMode();

  cyphal_feedback_sub = node_hdl.create_subscription<zubax::fluxgrip::Feedback_0_1>(FEEDBACK_PORT_ID, onFeedback_0_1_Received);
  delay(1000);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_cyphal_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(fg40_interfaces, msg, FG40MagnetCmd),
    "fg40_cmd"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(fg40_interfaces, msg, FG40Feedback),
    "fg40_status"));

  // create timer,
  const unsigned int timer_timeout = 1300;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &magnet_cmd_msg, &subscription_callback, ON_NEW_DATA));

  feedback_msg.magnetized = false;
  feedback_msg.remagnetization_state = 0;
  feedback_msg.cycles_on_off[0] = 0;
  feedback_msg.cycles_on_off[1] = 0;
}

void loop() {

  // while(digitalRead(MKRCAN_MCP2515_INT_PIN) == LOW)
  //   mcp2515.onExternalEventHandler();

  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();

  if(now - prev > 1000)
  {
    prev = now;

    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    cyphal_heartbeat_pub->publish(msg);
  }

  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}

