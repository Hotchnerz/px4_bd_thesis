/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal-Support/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal-Support.h>

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  Serial.println("Trigger sync reset in 5 seconds ...");
  auto const rc = cyphal::support::platform::reset_sync(std::chrono::milliseconds(5000));
  if (rc.has_value()) {
    Serial.print("reset_sync failed with error code ");
    Serial.println(static_cast<int>(rc.value()));
  }
}

void loop()
{

}
