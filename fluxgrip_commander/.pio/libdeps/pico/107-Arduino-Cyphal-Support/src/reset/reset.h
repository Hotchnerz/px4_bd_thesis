/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal-Support/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#ifdef ARDUINO_ARCH_RENESAS
# undef abs
#endif
#include <chrono>
#include <optional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal::support::platform
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class Error
{
  InvalidParam,
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

/* Immediately after calling this method a reset is performed. */
std::optional<Error> reset()
#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RENESAS)
__attribute__ ((error("Currently the reset API only supports ARDUINO_ARCH_RP2040 and ARDUINO_ARCH_RENESAS.")))
#endif
;

/* This method performs a reset in 'ms' milliseconds after its invocation,
 * blocking while waiting for the time to expire.
 */
std::optional<Error> reset_sync(std::chrono::milliseconds const ms)
#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RENESAS)
__attribute__ ((error("Currently the reset API only supports ARDUINO_ARCH_RP2040 and ARDUINO_ARCH_RENESAS.")))
#endif
;

/* This method performs a reset in 'ms' milliseconds after its invocation,
 * but returns immediately after its invocation.
 */
std::optional<Error> reset_async(std::chrono::milliseconds const ms)
#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RENESAS)
__attribute__ ((error("Currently the reset API only supports ARDUINO_ARCH_RP2040 and ARDUINO_ARCH_RENESAS.")))
#endif
;

/* Returns true if currently an async reset is pending. Since this
 * functionality is implemented via watchdog it is necessary to stop
 * regular watchdog feeding i.e. in the main loop.
 */
bool is_async_reset_pending()
#if !defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_RENESAS)
__attribute__ ((error("Currently the reset API only supports ARDUINO_ARCH_RP2040 and ARDUINO_ARCH_RENESAS.")))
#endif
;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* cyphal::support::platform */
