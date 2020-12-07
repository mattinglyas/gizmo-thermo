#include <Arduino.h>
#include <Wifi.h>
#include "ArduinoJson.h"
#include "FreeRTOS.h"
#include "AzureIotHub.h"
#include "ESP32MQTTClient.h"
#include "Config.h"
#include "PID_v1.h"
#include "analogWrite.h"
#include "CircularBuffer.h"

#define MESSAGE_MAX_LEN 4096 // size of message buffers
#define LOG_LEN 256 // max length of internal log
#define WIFI_TIMEOUT_MS 10000 // timeout for connecting to wifi network before re-trying

/* *******************************************************************************
 * Pin definitions
 * *******************************************************************************/

#define PIN_ONBOARD_LED 2
#define PIN_THERMISTOR_IN 32
#define PIN_PWM_OUT 4

/* *******************************************************************************
 * Typedefs
 * *******************************************************************************/

/* *******************************************************************************
 * Program constants
 * *******************************************************************************/

// TODO look into wifimanager library or similar solutions to prevent wifi settings from being leaked
static const char *ssid = CONFIG_WIFI_NAME;
static const char *password = CONFIG_WIFI_PASSWORD;

// string containing hostname, device id and device key
static const char *connectionString = CONFIG_CONNECTION_STRING;

// telemetry messages
const char *messageData = "{\"messageId\": %UL, \"temperature\": %D, \"dutyCycle\": %UD}"; // only sending telemetry messages as constant

// default configuration parameters
static unsigned long messageCount = 1;
static unsigned long interval = 2000;    // ms between telemetry messages
static bool messageSending = true;       // continuously send telemetry messages
static uint64_t lastMessage;             // storage for the last message sent (calls millis())
static bool ledValue = false;            // internal storage for value of onboard led
static double setpoint = 30;             // target setpoint for the heater
static double kP = 20, kI = 0.6, kD = 0; // initial parameters (best combination is kP = 20, kI = 0.4, kD = 0.5)
static double input, output;             // storage for measured and calculated heater values

PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT);

// logging
static CircularBuffer<float, LOG_LEN> tempBuffer; 

// thread safety
SemaphoreHandle_t pidUpdateMutex;

// task handlers
TaskHandle_t commsTaskHandler;
TaskHandle_t pidTaskHandler;

/* *******************************************************************************
 * Device Utilities
 * *******************************************************************************/

static bool initWifi(int timeoutInMs)
{
  Serial.print(F("INFO: Attempting to connect to "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int endTime = millis() + timeoutInMs;

  do
  {
    //delay(500);
    //Serial.print(".");
    delay(50);
    if (millis() > endTime) 
    {
      Serial.println(F("ERROR: Could not connect to WiFi"));
      return false;
    }
  } while (WiFi.status() != WL_CONNECTED);

  Serial.print(F("INFO: WiFi connected. Local IP Address: "));
  Serial.println(WiFi.localIP());
  return true;
}

static double readTemperature()
{
  // NOTE that these values are simply the scaled values (due to 4x resolution ADC) from PID_Basic_Thermistor_Calibration.ino
  double reading, result;

  xSemaphoreTake(pidUpdateMutex, portMAX_DELAY);

  // stop PWM output for reading
  analogWrite(PIN_PWM_OUT, 0);

  reading = 4096 - (double)analogRead(PIN_THERMISTOR_IN);

  /*
  reading = reading + 228; // heuristic offset so that input always > 0
  reading = reading * 900;
  reading = reading / 4096;
  */

  // curve fit 
  result = 4.51527 * exp(0.000735 * reading);

  // restart PWM output after reading
  analogWrite(PIN_PWM_OUT, 255);

  xSemaphoreGive(pidUpdateMutex);

  return result;
}

/* *******************************************************************************
 * Azure Utilities
 * *******************************************************************************/

static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    Serial.println(F("INFO: Send confirmation callback finished"));
  }
}

static void messageCallback(const char *payload, int size)
{
  Serial.print(F("INFO: Received message callback :"));
  Serial.println(payload);
}

static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    Serial.println(F("ERROR: Device twin callback unsuccessful; memory full!"));
    return;
  }
  memcpy(temp, payload, size);
  temp[size] = '\0';
  Serial.print(F("INFO: Received device twin callback: "));
  Serial.println(temp);
  free(temp);
}

static int deviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *responseSize)
{
  Serial.print(F("INFO: Received direct method for: "));
  Serial.println(methodName);

  StaticJsonDocument<MESSAGE_MAX_LEN> responseDoc;
  JsonObject responsePayload = responseDoc.createNestedObject("payload");

  int result = 200;

  StaticJsonDocument<MESSAGE_MAX_LEN> doc;
  deserializeJson(doc, payload);
  Serial.print(F("INFO: Received payload: "));
  serializeJson(doc, Serial);
  Serial.println();

  if (strcmp(methodName, "start") == 0)
  {
    messageSending = true;

    Serial.println(F("INFO: Started sending telemetry messages"));
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    messageSending = false;

    Serial.println(F("INFO: Stopped sending telemetry messages"));
  }
  else if (strcmp(methodName, "led") == 0)
  {
    ledValue = !ledValue;
    digitalWrite(PIN_ONBOARD_LED, ledValue);

    Serial.println(F("INFO: Toggled on-board LED"));
  }
  else if (strcmp(methodName, "interval") == 0)
  {
    JsonVariant newInterval = doc["interval"];
    interval = newInterval.as<int>();

    Serial.print(F("INFO: Changed telemetry interval to "));
    Serial.print(interval);
    Serial.println(F("ms"));
  }
  else if (strcmp(methodName, "setSetpoint") == 0)
  {
    JsonVariant newSetpoint = doc["setpoint"];
    setpoint = newSetpoint.as<double>();

    Serial.print(F("INFO: Changed telemetry interval to "));
    Serial.print(interval);
    Serial.println(F("ms"));
  }
  else if (strcmp(methodName, "getSetpoint") == 0)
  {
    responsePayload["setpoint"] = setpoint;

    Serial.println(F("INFO: Responding with setpoint value"));
  }
  else if (strcmp(methodName, "setPID") == 0)
  {
    JsonVariant newP = doc["kP"];
    JsonVariant newI = doc["kI"];
    JsonVariant newD = doc["kD"];

    // take mutex to prevent pid compute operations during value updating
    xSemaphoreTake(pidUpdateMutex, portMAX_DELAY);

    kP = newP.as<double>();
    kI = newI.as<double>();
    kD = newD.as<double>();
    pid.SetTunings(kP, kI, kD); 

    xSemaphoreGive(pidUpdateMutex);

    Serial.print(F("INFO: Updating PID parameters to kP = "));
    Serial.print(newP.as<double>());
    Serial.print(F(", kI = "));
    Serial.print(newI.as<double>());
    Serial.print(F(", kD = "));
    Serial.println(newD.as<double>());
  }
  else if (strcmp(methodName, "getPID") == 0)
  {
    responsePayload["kP"] = kP;
    responsePayload["kI"] = kI;
    responsePayload["kD"] = kD;

    Serial.println(F("INFO: Responding with PID k values"));
  }
  else if (strcmp(methodName, "getHistoricalData") == 0)
  {
    int i = 0;
    JsonArray data = responsePayload.createNestedArray("data");
    
    for (i = 0; i < tempBuffer.size(); i++) {
      data.add(tempBuffer[i]);
    }

    Serial.println(F("INFO: Responding with historical data"));
  }
  else 
  {
    // invalid direct method
    result = 404;
    Serial.println(F("WARNING: Received invalid command"));
  }

  responseDoc["result"] = result;

  char * serializedResponse = (char*)malloc(MESSAGE_MAX_LEN);
  serializeJson(responseDoc, serializedResponse, MESSAGE_MAX_LEN);

  (*response) = (unsigned char *)serializedResponse; // should not affect anything IF we only use chars <128
  (*responseSize) = strlen((const char *)serializedResponse);

  return result;
}

/* *******************************************************************************
 * Tasks
 * *******************************************************************************/

static void commsTask(void *pvParameters) 
{
  Serial.print(F("INFO: Stating messaging task on core "));
  Serial.println(xPortGetCoreID());

  // initialize wifi module
  bool hasWifi = false;
  do
  {
    hasWifi = initWifi(WIFI_TIMEOUT_MS);
  } while (!hasWifi);

  Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "Gizmo-Thermo");
  Esp32MQTTClient_Init((const uint8_t *) connectionString, true);

  Esp32MQTTClient_SetSendConfirmationCallback(sendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(messageCallback);
  Esp32MQTTClient_SetDeviceTwinCallback(deviceTwinCallback);
  Esp32MQTTClient_SetDeviceMethodCallback(deviceMethodCallback);

  lastMessage = millis();

  // forever
  for(;;)
  {
    /*  1. If I have wifi, check if it is time to send telemetry messages
     *  2. If it is time, send a message to azure and update the next telemetry time
     *  3. If we do not have wifi, reconnect until we get a valid connection to Azure
     */ 
    if (WiFi.status() == WL_CONNECTED)
    {
      if (messageSending && (int)(millis() - lastMessage) >= interval)
      {
        char messagePayload[MESSAGE_MAX_LEN];
        float temp = readTemperature();
        StaticJsonDocument<MESSAGE_MAX_LEN> doc;

        tempBuffer.push(temp);

        doc["messageId"] = messageCount;
        doc["temperature"] = temp;
        doc["dutyCycle"] = output;

        serializeJson(doc, messagePayload);

        Serial.print(F("INFO: Sending message to IoT Hub: "));
        Serial.println(messagePayload);

        EVENT_INSTANCE *message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
        Esp32MQTTClient_SendEventInstance(message);

        messageCount++;
        lastMessage = millis();
      }

      Esp32MQTTClient_Check();
    }
    else 
    {
      initWifi(WIFI_TIMEOUT_MS);
    }
  }

  vTaskDelay(100);
}

static void pidTask(void *pvParameters) 
{
  // initialize the variables that have been linked to the PID controller
  input = readTemperature();

  // turn the PID on
  pid.SetMode(AUTOMATIC);

  // forever
  for (;;) 
  {
    input = readTemperature();

    // take mutex to prevent values from updating during pid compute operations
    xSemaphoreTake(pidUpdateMutex, portMAX_DELAY);

    pid.Compute();
    analogWrite(PIN_PWM_OUT, output);

    xSemaphoreGive(pidUpdateMutex);

    vTaskDelay(100);
  }
}

void setup()
{
  pinMode(PIN_THERMISTOR_IN, INPUT);
  pinMode(PIN_ONBOARD_LED, OUTPUT);
  pinMode(PIN_PWM_OUT, OUTPUT);
  
  digitalWrite(PIN_ONBOARD_LED, HIGH);

  // begin setup
  Serial.begin(115200);
  Serial.println(F("*** Gizmo 6 ***"));
  Serial.println(F("INFO: Initializing..."));

  pidUpdateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
      commsTask,
      "Communications Task",
      65536,
      NULL,
      1,
      &commsTaskHandler,
      0);

  xTaskCreatePinnedToCore(
      pidTask,
      "PID Controller Task",
      16384,
      NULL,
      1,
      &pidTaskHandler,
      1);

  // finish setup
  digitalWrite(PIN_ONBOARD_LED, LOW);
}

void loop()
{
  // do nothing in the loop task
  vTaskSuspend(NULL);
}