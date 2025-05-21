<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `107-Arduino-Cyphal-Support`
==========================================
<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
[![Arduino Library Badge](https://www.ardu-badge.com/badge/107-Arduino-Cyphal-Support.svg?)](https://www.ardu-badge.com/107-Arduino-Cyphal-Support)
[![Compile Examples](https://github.com/107-systems/107-Arduino-Cyphal-Support/workflows/Compile%20Examples/badge.svg)](https://github.com/107-systems/107-Arduino-Cyphal-Support/actions?workflow=Compile+Examples)
[![Arduino Lint](https://github.com/107-systems/107-Arduino-Cyphal-Support/workflows/Arduino%20Lint/badge.svg)](https://github.com/107-systems/107-Arduino-Cyphal-Support/actions?workflow=Arduino+Lint)
[![keywords.txt Checks](https://github.com/107-systems/107-Arduino-Cyphal-Support/workflows/Extra%20Library%20Checks/badge.svg)](https://github.com/107-systems/107-Arduino-Cyphal-Support/actions?workflow=Extra+Library+Checks)
[![General Formatting Checks](https://github.com/107-systems/107-Arduino-Cyphal-Support/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/107-Arduino-Cyphal-Support/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/107-Arduino-Cyphal-Support/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/107-Arduino-Cyphal-Support/actions?workflow=Spell+Check)

This library provides support functionality for building a complete [Cyphal](https://opencyphal.org/) application in combination with [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal).

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="30%"></a>
  <a href="https://github.com/107-systems/viper"><img src="https://github.com/107-systems/.github/raw/main/logo/viper.jpg" width="30%"></a>
</p>

This library works for
* [arduino-pico](https://github.com/earlephilhower/arduino-pico): [`Raspberry Pi Pico`](https://www.raspberrypi.org/products/raspberry-pi-pico), `Adafruit Feather RP2040`, ... :heavy_check_mark:
* [ArduinoCore-renesas](https://github.com/arduino/ArduinoCore-renesas): [`Portenta C33`](https://store.arduino.cc/products/portenta-c33), [`Uno R4 WiFi`](https://store.arduino.cc/products/uno-r4-wifi), [`Uno R4 Minima`](https://store.arduino.cc/products/uno-r4-minima), ... :heavy_check_mark:

**Features:**
* API for obtaining a **unique 64-bit ID**.
```C++
auto /* std::array<uint8_t, 16> */ const UNIQUE_ID = cyphal::support::UniqueId::instance().value();
```
* API for **persistent register storage and retrieval**.
```C++
/* Declaration of key/value storage. */
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);

/* Load persistently stored registers from a non-volatile memory (EEPROM, flash, etc.). */
if (auto const opt_err = cyphal::support::load(kv_storage, *node_registry); opt_err.has_value())
{
  Serial.print("load failed with error code ");
  Serial.println(static_cast<int>(opt_err.value()));
}

/* Store persistent registers to a non-volatile memory (EEPROM, flash, etc.). */
if (auto const opt_err = cyphal::support::save(kv_storage, *node_registry); opt_err.has_value())
{
  Serial.print("save failed with error code ");
  Serial.println(static_cast<int>(opt_err.value()));
}
```
* API for performing **synchronous and asynchronous resets**.
```C++
/* Synchronous reset: */
cyphal::support::platform::reset_sync(std::chrono::milliseconds(5000));
/* Asynchronous reset: */
cyphal::support::platform::reset_async(std::chrono::milliseconds(5000));
```
