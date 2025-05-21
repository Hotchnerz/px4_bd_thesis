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

#if defined(ARDUINO_ARCH_RENESAS)

#include <WDT.h>

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

#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)
static uint32_t const WATCHDOG_MAX_DELAY_ms = 5592;
#elif defined(ARDUINO_PORTENTA_C33)
static uint32_t const WATCHDOG_MAX_DELAY_ms = 2684;
#else
# error "Your selected Renesas MCU is not supported"
#endif

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static bool is_async_reset_pending_flag = false;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

std::optional<Error> reset()
{
  WDT.begin(1); /* Minimum delay is 1 ms. */
  return std::nullopt;
}

std::optional<Error> reset_sync(std::chrono::milliseconds const ms)
{
  if (ms.count() > WATCHDOG_MAX_DELAY_ms)
    return Error::InvalidParam;

  WDT.begin(ms.count());
  for(;;) { delay(100); } /* Wait for the watchdog to bite. */
  return std::nullopt;
}

std::optional<Error> reset_async(std::chrono::milliseconds const ms)
{
  if (ms.count() > WATCHDOG_MAX_DELAY_ms)
    return Error::InvalidParam;

  WDT.begin(ms.count());

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

#endif /* defined(ARDUINO_ARCH_RENESAS) */
