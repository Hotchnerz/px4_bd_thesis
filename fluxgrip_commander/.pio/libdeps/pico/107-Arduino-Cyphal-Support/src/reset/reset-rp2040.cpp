/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal-Support/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "reset.h"

#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)

#include "hardware/watchdog.h"

/* Provide prototype for Arduino's delay function. */
extern "C" void delay(unsigned long);

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal::support::platform
{

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint32_t const RP2040_MAX_DELAY_ms = 0x7FFFFF;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static bool is_async_reset_pending_flag = false;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

std::optional<Error> reset()
{
  watchdog_enable(/* delay_ms */ 0, /* pause_on_debug */ true);
  return std::nullopt;
}

std::optional<Error> reset_sync(std::chrono::milliseconds const ms)
{
  if (ms.count() > RP2040_MAX_DELAY_ms)
    return Error::InvalidParam;

  watchdog_enable(/* delay_ms */ ms.count(), /* pause_on_debug */ true);
  for(;;) { delay(100); } /* Wait for the watchdog to bite. */
  return std::nullopt;
}

std::optional<Error> reset_async(std::chrono::milliseconds const ms)
{
  if (ms.count() > RP2040_MAX_DELAY_ms)
    return Error::InvalidParam;

  watchdog_enable(/* delay_ms */ ms.count(), /* pause_on_debug */ true);

  is_async_reset_pending_flag = true;

  return std::nullopt;
}

bool is_async_reset_pending()
{
  return is_async_reset_pending_flag;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* cyphal::support::platform */

#endif /* defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED) */
