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

  Serial.println(cyphal::support::UniqueId::instance());

  auto const UNIQUE_ID = cyphal::support::UniqueId::instance().value();

  Serial.print("Unique Id (64-Bit) = ");
  for (auto byte : UNIQUE_ID) {
    char msg[8] = {0};
    snprintf(msg, sizeof(msg), "%02X", byte);
    Serial.print(msg);
  }
  Serial.println();
}

void loop()
{

}
