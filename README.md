# OnlineLab - Thermo PID Controller

Modified version of example sketch demonstrating ability to send and receive coordinate data from Azure IoT hub. From https://docs.microsoft.com/en-us/samples/azure-samples/esp32-iot-devkit-get-started/sample/

Designed for the ENEE101 Covid19 Response Project. For more information, see University of Maryland website: https://ece.umd.edu/news/story/professor-romel-gomez-and-students-participate-in-iotswc20-digital-summit

## Setup

### In Platformio

1. Clone the entire repository into a folder 
2. Open project using Platformio using any editor
3. Build project
4. Download any missing libraries using "pio lib install <>", replacing <> with the name of any library missing
5. Go to config setup

### In Arduino IDE

1. Download main.cpp and rename to main.ino
2. Copy the folders "ArduinoJson", "ArrayQueue", "ESP32 Azure IoT Arduino" and "FreeRTOS" from ./lib/ into Arduino/libraries folder 
3. Replace \#include "config.h" with #include "../lib/config/src/config.h" in main.ino
4. Download any missing libraries using the Arduino Library Manager
5. Go to config setup

### Config Setup
These steps assume that there is a functional Azure IoT Hub and device already set up. Instructions to do so can be found at https://docs.microsoft.com/en-us/azure/iot-hub/iot-hub-create-through-portal

1. Go to Azure IoT Hub and go to the "IoT Devices" tab
2. Click on the device name that you wish to assign
3. Copy the primary connection string or secondary connection string into the CONFIG_CONNECTION_STRING define in config.h
4. Add WiFi connection details to CONFIG_WIFI_NAME and CONFIG_WIFI_PASSWORD

## Direct Methods

### Start

Causes device to resume sending telemetry messages

#### Payload

None

#### Example Message

```
{
    "methodName": "start",
    "responseTimeoutInSeconds": 200,
    "payload" : {}
}
```

### Stop

Causes device to halt sending telemetry messages

#### Payload

None

#### Example Message

```
{
    "methodName": "start",
    "responseTimeoutInSeconds": 200,
    "payload" : {}
}
```

### LED

Toggles the state of the on-board LED assigned to pin 2

#### Payload

None

#### Example Message

```
{
    "methodName": "led",
    "responseTimeoutInSeconds": 200,
    "payload" : {}
}
```

### Interval

Changes the delay between telemetry messages in milliseconds

#### Payload

Positive integer value associated with "interval" to be used as new delay

#### Example Message

```
{
    "methodName": "interval",
    "responseTimeoutInSeconds": 200,
    "payload" : {"interval": 2000}
}
```

### Set Setpoint

Changes the setpoint of the PID controller (in degrees C)

#### Payload 

Positive integer value associated with "setpoint" to be used as new value

#### Example Message
```
{
    "methodName": "setSetpoint",
    "responseTimeoutInSeconds": 200,
    "payload" : {"setpoint": 60}
}
```

### Set PID Values

Changes the PID parameters of the PID controller

#### Payload 

Positive integer value associated with "kP", "kI", "kD" to be used as new value

#### Example Message
```
{
    "methodName": "setPID",
    "responseTimeoutInSeconds": 200,
    "payload" : {"kP": 0.6,"kI": 0.25, "kD": 0}
}
```

### Get PID Values

Responds with the PID parameters

#### Payload 

None

#### Example Message
```
{
    "methodName": "getPID",
    "responseTimeoutInSeconds": 200,
    "payload" : {}
}
```

#### Example Response
```
{
    "result":200,
    "payload" : {"kP": 0.6,"kI": 0.25, "kD": 0}
}
```

### Get Historical Data

Responds with up to LOG_LEN historical temperature readings

#### Payload 

None

#### Example Message
```
{
    "methodName": "getHistoricalData",
    "responseTimeoutInSeconds": 200,
    "payload" : {}
}
```

#### Example Response
```
{
    "result":200,
    "payload" : {"data":[50.0, 50.01, 50.02, 55, 56, 57]}
}
```
