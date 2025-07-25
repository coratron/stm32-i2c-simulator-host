# ESP32-C3 I2C Tester for STM32 I2C Simulator

This ESP32-C3 project tests the STM32 I2C device simulator by acting as an I2C master.

## Features

1. **I2C Bus Scanner**: Continuously scans for devices on the I2C bus
2. **Device-Specific Tests**:
   - MCP23017 (x2): Tests I/O direction and GPIO read/write
   - ADS1015: Tests configuration and ADC conversion
   - DS3231: Reads time/date and temperature

## Hardware Connections

Connect the ESP32-C3 to the STM32G030F6P6:

| ESP32-C3 | STM32G030F6P6 | Description |
|----------|---------------|-------------|
| GPIO9    | PB6           | I2C SCL     |
| GPIO8    | PB7           | I2C SDA     |
| GND      | GND           | Ground      |

**Note**: Both boards already have pull-up resistors configured, so no external pull-ups are needed.

## Building and Flashing

1. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. Set target to ESP32-C3:
   ```bash
   idf.py set-target esp32c3
   ```

3. Build the project:
   ```bash
   idf.py build
   ```

4. Flash and monitor:
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## Expected Output

The serial monitor will show:

1. **I2C Bus Scan** (every 5 seconds):
   - Shows all detected I2C addresses
   - Expected devices at 0x20, 0x21, 0x48, and 0x68

2. **Device Tests** (every 10 seconds):
   - MCP23017 register read/write tests
   - ADS1015 configuration and conversion
   - DS3231 time, date, and temperature readings

## Testing Procedure

1. Flash the STM32 simulator first
2. Flash this ESP32-C3 tester
3. Connect the I2C lines and ground
4. Monitor the ESP32-C3 serial output
5. Verify all 4 devices are detected and responding correctly

## Troubleshooting

- If no devices detected: Check wiring and ground connection
- If intermittent detection: Check power supply stability
- If wrong data: Verify STM32 is running at 64MHz