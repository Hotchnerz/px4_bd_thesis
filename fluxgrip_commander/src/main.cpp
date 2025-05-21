/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <Arduino.h>
#include <ros.h>
#include <stdio.h>
#include <fg40_msgs/FG40MagnetCmd.h>
#include <fg40_msgs/FG40Feedback.h>
#include <SPI.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-CriticalSection.h>
#include <107-Arduino-Cyphal-Support.h>

/**************************************************************************************
 * ROS SERIAL GLOBALS
 **************************************************************************************/

fg40_msgs::FG40MagnetCmd magnet_cmd_msg;
fg40_msgs::FG40Feedback feedback_msg;
fg40_msgs::FG40Feedback feedback_prev;

ros::NodeHandle nh;
ros::Publisher mag_status("fg40_status", &feedback_msg);
ros::Subscriber<fg40_msgs::FG40MagnetCmd> mag_cmd("fg40_cmd", &subscription_callback);


/**************************************************************************************
 * MCP2515 CONFIG / GLOBALS
 **************************************************************************************/

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

/**************************************************************************************
 * CYPHAL GLOBALS
 **************************************************************************************/

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

CanardPortID const CMD_PORT_ID = 1000U;
cyphal::Publisher<uavcan::primitive::scalar::Integer8_1_0> cyphal_cmd_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>
  (CMD_PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::Publisher<uavcan::node::Heartbeat_1_0> cyphal_heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>
  (1*1000*1000UL /* = 1 sec in usecs. */);

static CanardPortID const FEEDBACK_PORT_ID   = 1001U;
cyphal::Subscription cyphal_feedback_sub;

/**************************************************************************************
 * CYPHAL FUNCTIONS
 **************************************************************************************/

// Process frame when interrupt is pulled low
void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

// Callback for Cyphal subscriber after spin() matches the frame to the target node
void onFeedback_0_1_Received(zubax::fluxgrip::Feedback_0_1 const & recieved_fg40_msg)
{
  feedback_msg.magnetized = recieved_fg40_msg.magnetized;
  feedback_msg.remagnetization_state = recieved_fg40_msg.remagnetization_state;
  feedback_msg.cycles_on_off[0] = recieved_fg40_msg.cycles_on_off[0];
  feedback_msg.cycles_on_off[1] = recieved_fg40_msg.cycles_on_off[1];
}

/**************************************************************************************
 * MICRO ROS FUNCTIONS
 **************************************************************************************/

// Flash LED_BUILTIN if ROS functions encounter some sort of error
void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Callback for publisher
void timer_callback()
{  

    //If the FG40's last state was active but lost power, publish a force mag cyphal message to resync the state to this node. Otherwise, ignore.
    if ((feedback_prev.magnetized != feedback_msg.magnetized) && ((feedback_prev.cycles_on_off[0] != feedback_msg.cycles_on_off[0]) || ( feedback_prev.cycles_on_off[1] != feedback_msg.cycles_on_off[1]))){
      uavcan::primitive::scalar::Integer8_1_0 cyphal_msg;
      cyphal_msg.value = 2;
      cyphal_cmd_pub->publish(cyphal_msg);
    }

    //Update previous message state
    feedback_prev.magnetized = feedback_msg.magnetized;
    feedback_prev.remagnetization_state = feedback_msg.remagnetization_state;
    feedback_prev.cycles_on_off[0] = feedback_msg.cycles_on_off[0];
    feedback_prev.cycles_on_off[1] = feedback_msg.cycles_on_off[1];

  
}

// Listen to /fg40_cmd and publish cmd as a cyphal CAN message
void subscription_callback(const fg40_msgs::FG40MagnetCmd& msgin)
{  
  int cmd = msgin.cmd_magnet;
  uavcan::primitive::scalar::Integer8_1_0 cyphal_msg;

  switch(cmd){
    
    // Demagnetize FG40
    case 0:
    cyphal_msg.value = 0;
    mag_status.publish(&cyphal_msg);
    break;

    // Magnetize FG40
    case 1:
    cyphal_msg.value = 1;
    mag_status.publish(&cyphal_msg);
    break;

    // FORCE Magnetize/Demagnetize Cycle on FG40
    case 2:
    cyphal_msg.value = 2;
    mag_status.publish(&cyphal_msg);
    break;

    // Ignore other INT values and replace with a magnetize cmd
    default:
    cyphal_msg.value = 1;
    mag_status.publish(&cyphal_msg);
    break;

  }

}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup() {
    // Start Serial Connection
    Serial.begin(115200);

    // Use Node API and set info aboout this node. Uses the default node ID of 42
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
  /* Setbitrate for MCP2515. MUST USE 1000k! FG40 BY DEFAULT USES THIS AS THE DATA AND ARBITRATION RATE. 8 MHz due to crystal on MCP2515 Breakout*/
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_1000kBPS_8MHZ);
  mcp2515.setNormalMode();

  /* Instantiate cyphal subscriber here. Placing it in the decleration causes the pico to freeze and crash. */
  cyphal_feedback_sub = node_hdl.create_subscription<zubax::fluxgrip::Feedback_0_1>(FEEDBACK_PORT_ID, onFeedback_0_1_Received);
  delay(1000);

  nh.initNode();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  
  delay(2000);
  
  nh.publisher(mag_status);
  nh.subscribe(mag_cmd);

  // Instantiate callback message
  feedback_msg.magnetized = false;
  feedback_msg.remagnetization_state = 0;
  feedback_msg.cycles_on_off[0] = 0;
  feedback_msg.cycles_on_off[1] = 0;

  feedback_prev.magnetized = false;
  feedback_prev.remagnetization_state = 0;
  feedback_prev.cycles_on_off[0] = 0;
  feedback_prev.cycles_on_off[1] = 0;
}

void loop() {

  /* Process all pending OpenCyphal actions. */
  /* This block needs a semaphore or pico will deadlock*/
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

  /* Wait a bit and then execute ROS activities*/
  delay(100);
  timer_callback();
  nh.spinOnce();
}

