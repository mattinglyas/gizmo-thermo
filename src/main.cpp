#include <Arduino.h>
#include <Wifi.h>
#include "ArduinoJson.h"
#include "FreeRTOS.h"
#include "AzureIotHub.h"
#include "ESP32MQTTClient.h"
#include "Config.h"
#include "PID_v1.h"

#define MESSAGE_MAX_LEN 16384 // size of message buffers

#define WIFI_TIMEOUT_MS 10000 // timeout for connecting to wifi network before re-trying

/* *******************************************************************************
 * Pin definitions
 * *******************************************************************************/

#define ONBOARD_LED_PIN 2
#define THERMISTOR_IN_PIN 32
#define PWM_OUT_PIN 4

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
const char *responseMessageData = "{\"status\": %UD, \"payload\": %s}";                    // respond with a status code and payload

// default configuration parameters
static unsigned long messageCount = 1;
static unsigned long interval = 2000; // ms between telemetry messages
static bool messageSending = true;    // continuously send telemetry messages
static uint64_t lastMessage;          // storage for the last message sent (calls millis())
static bool ledValue = false;         // internal storage for value of onboard led
static int setpoint = 0;              // target setpoint for the heater
double kP = 20, kI = 0.6, kD = 0;     // initial parameters (best combination is kP = 20, kI = 0.4, kD = 0.5)

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

  char *responseMessage = (char *)malloc(MESSAGE_MAX_LEN);
  int result = 200;

  StaticJsonDocument<MESSAGE_MAX_LEN> doc;
  deserializeJson(doc, payload, MESSAGE_MAX_LEN);
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
    digitalWrite(ONBOARD_LED_PIN, ledValue);
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
    setpoint = newSetpoint.as<int>();
    Serial.print(F("INFO: Changed telemetry interval to "));
    Serial.print(interval);
    Serial.println(F("ms"));
  }
  else if (strcmp(methodName, "getSetpoint") == 0)
  {
    
  }
  else if (strcmp(methodName, "setPID") == 0)
  {
    JsonVariant newP = doc["p"];
    JsonVariant newI = doc["i"];
    JsonVariant newD = doc["d"];


  }
  else if (strcmp(methodName, "getPID") == 0)
  {

  }
  else if (strcmp(methodName, "getHistoricalData") == 0)
  {
    
  }
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

  while (true)
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


        // copy into message
        // TODO status messages
        
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

}

void setup()
{
  // put your setup code here, to run once:
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, true);
}

void loop()
{
  // put your main code here, to run repeatedly:
}