/*
DALY BMS to MQTT Project
https://github.com/softwarecrash/DALY-BMS-to-MQTT
This code is free for use without any waranty.
when copy code or reuse make a note where the codes comes from.


Dear programmer:
When I wrote this code, only god and
I knew how it worked.
Now, only god knows it!

Therefore, if you are trying to optimize
this routine and it fails (most surely),
please increase this counter as a
warning for the next person:

total_hours_wasted_here = 254
*/

#include "main.h"
#include <daly-bms-uart.h> // This is where the library gets pulled in

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

#include "Settings.h"

const char* ssid = "xxx";
const char* password = "xxx";

IPAddress local_IP(192, 168, 0, 36);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient client;
Settings _settings;
PubSubClient mqttclient(client);

StaticJsonDocument<JSON_BUFFER> bmsJson;                          // main Json
JsonObject deviceJson = bmsJson.createNestedObject("Device");     // basic device data
JsonObject packJson = bmsJson.createNestedObject("Pack");         // battery package data
JsonObject cellVJson = bmsJson.createNestedObject("CellV");       // nested data for cell voltages
JsonObject cellTempJson = bmsJson.createNestedObject("CellTemp"); // nested data for cell temp

int mqttdebug;

unsigned long mqtttimer = 0;
unsigned long bmstimer = 0;
unsigned long RestartTimer = 0;

Daly_BMS_UART bms(MYPORT_RX, MYPORT_TX);

#include "status-LED.h"

// flag for saving data and other things
bool shouldSaveConfig = false;
bool restartNow = false;
bool updateProgress = false;
bool dataCollect = false;
bool firstPublish = false;
unsigned long wakeuptimer = WAKEUP_INTERVAL; // dont run immediately after boot, wait for first intervall
bool wakeupPinActive = false;

unsigned long relaistimer = RELAISINTERVAL; // dont run immediately after boot, wait for first intervall
float relaisCompareValueTmp = 0;
bool relaisComparsionResult = false;

char mqttClientId[80];

ADC_MODE(ADC_VCC);

//----------------------------------------------------------------------
void saveConfigCallback()
{

  DEBUG_PRINTLN(F("Should save config"));
  shouldSaveConfig = true;
}

bool wakeupHandler()
{
  if (_settings.data.wakeupEnable && (millis() > wakeuptimer))
  {
    DEBUG_PRINTLN();
    DEBUG_PRINTLN(F("wakeupHandler()"));
    DEBUG_PRINT(F("this run:\t"));
    DEBUG_PRINTLN(millis());
    DEBUG_PRINT(F("next run:\t"));
    DEBUG_PRINTLN(wakeuptimer);
    if (wakeupPinActive)
    {
      wakeupPinActive = false;
      wakeuptimer = millis() + WAKEUP_INTERVAL;
      digitalWrite(WAKEUP_PIN, LOW);
    }
    else
    {
      wakeupPinActive = true;
      wakeuptimer = millis() + WAKEUP_DURATION;
      digitalWrite(WAKEUP_PIN, HIGH);
    }
    DEBUG_PRINT(F("PIN IS NOW:\t"));
    DEBUG_PRINTLN(digitalRead(WAKEUP_PIN));
  }
  return true;
}

bool relaisHandler()
{
  if (_settings.data.relaisEnable && (millis() - relaistimer > RELAISINTERVAL))
  {
    relaistimer = millis();
    // read the value to compare to depending on the mode
    switch (_settings.data.relaisFunction)
    {
    case 0:
      // Mode 0 - Lowest Cell Voltage
      relaisCompareValueTmp = bms.get.minCellmV / 1000;
      break;
    case 1:
      // Mode 1 - Highest Cell Voltage
      relaisCompareValueTmp = bms.get.maxCellmV / 1000;
      break;
    case 2:
      // Mode 2 - Pack Voltage
      relaisCompareValueTmp = bms.get.packVoltage;
      break;
    case 3:
      // Mode 3 - Temperature
      relaisCompareValueTmp = bms.get.tempAverage;
      break;
    case 4:
      // Mode 4 - Manual per WEB or MQTT
      break;
    }
    // if(relaisCompareValueTmp == NULL){
    if (relaisCompareValueTmp == '\0' && _settings.data.relaisFunction != 4)
    {
      if (_settings.data.relaisFailsafe)
      {
        return false;
      }
      else
      {
        relaisComparsionResult = false;
        _settings.data.relaisInvert ? digitalWrite(RELAIS_PIN, !relaisComparsionResult) : digitalWrite(RELAIS_PIN, relaisComparsionResult);
      }
    }
    // now compare depending on the mode
    if (_settings.data.relaisFunction != 4)
    {
      // other modes
      switch (_settings.data.relaisComparsion)
      {
      case 0:
        // Higher or equal than
        // check if value is already true so we have to use hysteresis to switch off
        if (relaisComparsionResult)
        {
          relaisComparsionResult = relaisCompareValueTmp >= (_settings.data.relaisSetValue - _settings.data.relaisHysteresis) ? true : false;
        }
        else
        {
          // check if value is greater than
          relaisComparsionResult = relaisCompareValueTmp >= (_settings.data.relaisSetValue) ? true : false;
        }
        break;
      case 1:
        // Lower or equal than
        // check if value is already true so we have to use hysteresis to switch off
        if (relaisComparsionResult)
        {
          // use hystersis to switch off
          relaisComparsionResult = relaisCompareValueTmp <= (_settings.data.relaisSetValue + _settings.data.relaisHysteresis) ? true : false;
        }
        else
        {
          // check if value is greater than
          relaisComparsionResult = relaisCompareValueTmp <= (_settings.data.relaisSetValue) ? true : false;
        }
        break;
      }
    }
    else
    {
      // manual mode, currently no need to set anything, relaisComparsionResult is set by WEB or MQTT
      // i keep this just here for better reading of the code. The else {} statement can be removed later
    }

    _settings.data.relaisInvert ? digitalWrite(RELAIS_PIN, !relaisComparsionResult) : digitalWrite(RELAIS_PIN, relaisComparsionResult);

    return true;
  }
  return false;
}

void setup()
{
  DEBUG_BEGIN(9600); // Debugging towards UART

  pinMode(WAKEUP_PIN, OUTPUT);
  pinMode(RELAIS_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);
  WiFi.persistent(true);                          // fix wifi save bug

    // Configures static IP address
    if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
    }

    // Connect to Wi-Fi network with SSID and password
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

  deviceJson["Name"] = _settings.data.deviceName; // set the device name in json string

  sprintf(mqttClientId, "%s-%06X", _settings.data.deviceName, ESP.getChipId());


  DEBUG_PRINTLN();
  DEBUG_PRINT(F("Device Name:\t"));
  DEBUG_PRINTLN(_settings.data.deviceName);
  DEBUG_PRINT(F("Mqtt Server:\t"));
  DEBUG_PRINTLN(_settings.data.mqttServer);
  DEBUG_PRINT(F("Mqtt Port:\t"));
  DEBUG_PRINTLN(_settings.data.mqttPort);
  DEBUG_PRINT(F("Mqtt User:\t"));
  DEBUG_PRINTLN(_settings.data.mqttUser);
  DEBUG_PRINT(F("Mqtt Passwort:\t"));
  DEBUG_PRINTLN(_settings.data.mqttPassword);
  DEBUG_PRINT(F("Mqtt Interval:\t"));
  DEBUG_PRINTLN(_settings.data.mqttRefresh);
  DEBUG_PRINT(F("Mqtt Topic:\t"));
  DEBUG_PRINTLN(_settings.data.mqttTopic);
  DEBUG_PRINT(F("wakeupEnable:\t"));
  DEBUG_PRINTLN(_settings.data.wakeupEnable);
  DEBUG_PRINT(F("relaisEnable:\t"));
  DEBUG_PRINTLN(_settings.data.relaisEnable);
  DEBUG_PRINT(F("relaisInvert:\t"));
  DEBUG_PRINTLN(_settings.data.relaisInvert);
  DEBUG_PRINT(F("relaisFunction:\t"));
  DEBUG_PRINTLN(_settings.data.relaisFunction);
  DEBUG_PRINT(F("relaisComparsion:\t"));
  DEBUG_PRINTLN(_settings.data.relaisComparsion);
  DEBUG_PRINT(F("relaisSetValue:\t"));
  DEBUG_PRINTLN(_settings.data.relaisSetValue);
  DEBUG_PRINT(F("relaisHysteresis:\t"));
  DEBUG_PRINTLN(_settings.data.relaisHysteresis);

  mqttclient.setServer(_settings.data.mqttServer, _settings.data.mqttPort);
  DEBUG_PRINTLN(F("MQTT Server config Loaded"));

  mqttclient.setCallback(mqttcallback);
  mqttclient.setBufferSize(MQTT_BUFFER);

  deviceJson["IP"] = WiFi.localIP(); // grab the device ip

  bms.Init(); // init the bms driver
  bms.callback(prozessUartData);

  analogWrite(LED_PIN, 255);
}
// end void setup
void loop()
{
  // Make sure wifi is in the right mode
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!updateProgress)
    {
      if (millis() >= (mqtttimer + (_settings.data.mqttRefresh * 1000)))
      {

        getJsonDevice();
        bms.update();
        if (bms.getState() >= 0)
        {
          getJsonData();
          sendtoMQTT();
          mqtttimer = millis();
        }
        else if (bms.getState() == -2)
        {
          getJsonData();
          sendtoMQTT();
          mqtttimer = millis();
        }
      }
    }
  }
  if (restartNow && millis() >= (RestartTimer + 500))
  {
    DEBUG_PRINTLN(F("Restart"));
    ESP.restart();
  }
  wakeupHandler();
  relaisHandler();

  notificationLED(); // notification LED routine
  mqttclient.loop(); // Check if we have something to read from MQTT
}
// End void loop
void prozessUartData()
{
  if (!updateProgress)
  {
    /*
    DEBUG_PRINTLN(F("Hello world as callback from uart!!!!!!!!!!!!!!!!!"));
    getJsonDevice();

    getJsonData();

    notifyClients();

    if (millis() > (mqtttimer + (_settings.data.mqttRefresh * 1000)))
    {
        sendtoMQTT();
        mqtttimer = millis();
    }
    */
  }
}

void getJsonDevice()
{
  deviceJson[F("ESP_VCC")] = ESP.getVcc() / 1000.0;
  deviceJson[F("Wifi_RSSI")] = WiFi.RSSI();
  deviceJson[F("Relais_Active")] = relaisComparsionResult ? true : false;
  deviceJson[F("Relais_Manual")] = _settings.data.relaisEnable && _settings.data.relaisFunction == 4 ? true : false;
  deviceJson[F("sw_version")] = SOFTWARE_VERSION;
#ifdef DALY_BMS_DEBUG
  deviceJson[F("CPU_Frequency")] = ESP.getCpuFreqMHz();
  deviceJson[F("Real_Flash_Size")] = ESP.getFlashChipRealSize();
  deviceJson[F("Flash_Size")] = ESP.getFlashChipSize();
  deviceJson[F("Sketch_Size")] = ESP.getSketchSize();
  deviceJson[F("Free_Sketch_Space")] = ESP.getFreeSketchSpace();
  deviceJson[F("Free_Heap")] = ESP.getFreeHeap();
  deviceJson[F("HEAP_Fragmentation")] = ESP.getHeapFragmentation();
  deviceJson[F("Free_BlockSize")] = ESP.getMaxFreeBlockSize();
  deviceJson[F("json_memory_usage")] = bmsJson.memoryUsage();
  deviceJson[F("json_capacity")] = bmsJson.capacity();
  deviceJson[F("runtime")] = millis() / 1000;
  deviceJson[F("ws_clients")] = 0;
  deviceJson[F("MQTT_Json")] = _settings.data.mqttJson;
#endif
}

void getJsonData()
{
  packJson[F("Voltage")] = bms.get.packVoltage;
  packJson[F("Current")] = bms.get.packCurrent;
  packJson[F("Power")] = (bms.get.packCurrent * bms.get.packVoltage);
  packJson[F("SOC")] = bms.get.packSOC;
  packJson[F("Remaining_mAh")] = bms.get.resCapacitymAh;
  packJson[F("Cycles")] = bms.get.bmsCycles;
  packJson[F("BMS_Temp")] = bms.get.tempAverage;
  packJson[F("Cell_Temp")] = bms.get.cellTemperature[0];
  packJson[F("High_CellNr")] = bms.get.maxCellVNum;
  packJson[F("High_CellV")] = bms.get.maxCellmV / 1000;
  packJson[F("Low_CellNr")] = bms.get.minCellVNum;
  packJson[F("Low_CellV")] = bms.get.minCellmV / 1000;
  packJson[F("Cell_Diff")] = bms.get.cellDiff;
  packJson[F("DischargeFET")] = bms.get.disChargeFetState ? true : false;
  packJson[F("ChargeFET")] = bms.get.chargeFetState ? true : false;
  packJson[F("Status")] = bms.get.chargeDischargeStatus;
  packJson[F("Cells")] = bms.get.numberOfCells;
  packJson[F("Heartbeat")] = bms.get.bmsHeartBeat;
  packJson[F("Balance_Active")] = bms.get.cellBalanceActive ? true : false;

  for (size_t i = 0; i < size_t(bms.get.numberOfCells); i++)
  {
    cellVJson[F("CellV_") + String(i + 1)] = bms.get.cellVmV[i] / 1000;
    cellVJson[F("Balance_") + String(i + 1)] = bms.get.cellBalanceState[i];
  }

  for (size_t i = 0; i < size_t(bms.get.numOfTempSensors); i++)
  {
    cellTempJson[F("Cell_Temp_") + String(i + 1)] = bms.get.cellTemperature[i];
  }
}

char *topicBuilder(char *buffer, char const *path, char const *numering = "")
{                                                   // buffer, topic
  const char *mainTopic = _settings.data.mqttTopic; // get the main topic path

  strcpy(buffer, mainTopic);
  strcat(buffer, "/");
  strcat(buffer, path);
  strcat(buffer, numering);
  return buffer;
}

bool sendtoMQTT()
{
  char msgBuffer[32];
  char buff[256]; // temp buffer for the topic string
  if (!connectMQTT())
  {
    DEBUG_PRINTLN(F("Error: No connection to MQTT Server, cant send Data!"));
    firstPublish = false;
    return false;
  }
  DEBUG_PRINT(F("Info: Data sent to MQTT Server... "));
  if (!_settings.data.mqttJson)
  {
    mqttclient.publish(topicBuilder(buff, "Pack_Voltage"), dtostrf(bms.get.packVoltage, 4, 1, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_Current"), dtostrf(bms.get.packCurrent, 4, 1, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_Power"), dtostrf((bms.get.packVoltage * bms.get.packCurrent), 4, 1, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_SOC"), dtostrf(bms.get.packSOC, 6, 2, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_Remaining_mAh"), itoa(bms.get.resCapacitymAh, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Cycles"), itoa(bms.get.bmsCycles, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_BMS_Temperature"), itoa(bms.get.tempAverage, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Cell_High"), itoa(bms.get.maxCellVNum, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Cell_Low"), itoa(bms.get.minCellVNum, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Cell_High_Voltage"), dtostrf(bms.get.maxCellmV / 1000, 5, 3, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_Cell_Low_Voltage"), dtostrf(bms.get.minCellmV / 1000, 5, 3, msgBuffer));
    mqttclient.publish(topicBuilder(buff, "Pack_Cell_Difference"), itoa(bms.get.cellDiff, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_ChargeFET"), bms.get.chargeFetState ? "true" : "false");
    mqttclient.publish(topicBuilder(buff, "Pack_DischargeFET"), bms.get.disChargeFetState ? "true" : "false");
    mqttclient.publish(topicBuilder(buff, "Pack_Status"), bms.get.chargeDischargeStatus);
    mqttclient.publish(topicBuilder(buff, "Pack_Cells"), itoa(bms.get.numberOfCells, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Heartbeat"), itoa(bms.get.bmsHeartBeat, msgBuffer, 10));
    mqttclient.publish(topicBuilder(buff, "Pack_Balance_Active"), bms.get.cellBalanceActive ? "true" : "false");

    for (size_t i = 0; i < bms.get.numberOfCells; i++)
    {
      mqttclient.publish(topicBuilder(buff, "Pack_Cells_Voltage/Cell_", itoa((i + 1), msgBuffer, 10)), dtostrf(bms.get.cellVmV[i] / 1000, 5, 3, msgBuffer));
      mqttclient.publish(topicBuilder(buff, "Pack_Cells_Balance/Cell_", itoa((i + 1), msgBuffer, 10)), bms.get.cellBalanceState[i] ? "true" : "false");
    }
    for (size_t i = 0; i < bms.get.numOfTempSensors; i++)
    {
      mqttclient.publish(topicBuilder(buff, "Pack_Cell_Temperature_", itoa((i + 1), msgBuffer, 10)), itoa(bms.get.cellTemperature[i], msgBuffer, 10));
    }
    mqttclient.publish(topicBuilder(buff, "RelaisOutput_Active"), relaisComparsionResult ? "true" : "false");
    mqttclient.publish(topicBuilder(buff, "RelaisOutput_Manual"), (_settings.data.relaisFunction == 4) ? "true" : "false"); // should we keep this? you can check with iobroker etc. if you can even switch the relais using mqtt
  }
  else
  {
    char data[JSON_BUFFER];
    serializeJson(bmsJson, data);
    mqttclient.setBufferSize(JSON_BUFFER + 100);
    mqttclient.publish(topicBuilder(buff, "Pack_Data"), data, false);
  }
  DEBUG_PRINT(F("Done\n"));
  firstPublish = true;

  return true;
}

void mqttcallback(char *topic, unsigned char *payload, unsigned int length)
{
  char buff[256];
  if (firstPublish == false)
    return;

  updateProgress = true;

  String messageTemp;
  for (unsigned int i = 0; i < length; i++)
  {
    messageTemp += (char)payload[i];
  }

  // check if the message not empty
  if (messageTemp.length() <= 0)
  {
    DEBUG_PRINTLN(F("MQTT Callback: message empty, break!"));
    updateProgress = false;
    return;
  }
  DEBUG_PRINTLN(F("MQTT Callback: message recived: ") + messageTemp);
  // set Relais
  if (strcmp(topic, topicBuilder(buff, "Device_Control/Relais")) == 0)
  {
    if (_settings.data.relaisFunction == 4 && messageTemp == "true")
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Relais on"));
      relaisComparsionResult = true;
      mqttclient.publish(topicBuilder(buff, "Device_Control/Relais"), "true", false);
      relaisHandler();
    }
    if (_settings.data.relaisFunction == 4 && messageTemp == "false")
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Relais off"));
      relaisComparsionResult = false;
      mqttclient.publish(topicBuilder(buff, "Device_Control/Relais"), "false", false);
      relaisHandler();
    }
  }
  // set SOC
  if (strcmp(topic, topicBuilder(buff, "Device_Control/Pack_SOC")) == 0)
  {
    if (bms.get.packSOC != atof(messageTemp.c_str()) && atof(messageTemp.c_str()) >= 0 && atof(messageTemp.c_str()) <= 100)
    {
      if (bms.setSOC(atof(messageTemp.c_str())))
      {
        DEBUG_PRINTLN(F("MQTT Callback: SOC message OK, Write: ") + messageTemp);
        mqttclient.publish(topicBuilder(buff, "Device_Control/Pack_SOC"), String(atof(messageTemp.c_str())).c_str(), false);
      }
    }
  }

  // Switch the Discharging port
  if (strcmp(topic, topicBuilder(buff, "Device_Control/Pack_DischargeFET")) == 0)
  {
    if (messageTemp == "true" && !bms.get.disChargeFetState)
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Discharging mos on"));
      bms.setDischargeMOS(true);
      mqttclient.publish(topicBuilder(buff, "Device_Control/Pack_DischargeFET"), "true", false);
    }
    if (messageTemp == "false" && bms.get.disChargeFetState)
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Discharging mos off"));
      bms.setDischargeMOS(false);
      mqttclient.publish(topicBuilder(buff, "Device_Control/Pack_DischargeFET"), "false", false);
    }
  }

  // Switch the Charging Port
  if (strcmp(topic, topicBuilder(buff, "Device_Control/Pack_ChargeFET")) == 0)
  {
    DEBUG_PRINTLN(F("message recived: ") + messageTemp);

    if (messageTemp == "true" && !bms.get.chargeFetState)
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Charging mos on"));
      bms.setChargeMOS(true);
      mqttclient.publish(topicBuilder(buff, "Device_Control/Pack_ChargeFET"), "true", false);
    }
    if (messageTemp == "false" && bms.get.chargeFetState)
    {
      DEBUG_PRINTLN(F("MQTT Callback: switching Charging mos off"));
      bms.setChargeMOS(false);
      mqttclient.publish(topicBuilder(buff, "Device_Control/Pack_ChargeFET"), "false", false);
    }
  }
  updateProgress = false;
}

bool connectMQTT()
{
  char buff[256];
  if (!mqttclient.connected())
  {
    firstPublish = false;
    DEBUG_PRINT(F("Info: MQTT Client State is: "));
    DEBUG_PRINTLN(mqttclient.state());
    DEBUG_PRINT(F("Info: establish MQTT Connection... "));

    if (mqttclient.connect(mqttClientId, _settings.data.mqttUser, _settings.data.mqttPassword, (topicBuilder(buff, "alive")), 0, true, "false", true))
    {
      if (mqttclient.connected())
      {
        DEBUG_PRINT(F("Done\n"));
        mqttclient.publish(topicBuilder(buff, "alive"), "true", true); // LWT online message must be retained!
        mqttclient.publish(topicBuilder(buff, "Device_IP"), (const char *)(WiFi.localIP().toString()).c_str(), true);
        mqttclient.subscribe(topicBuilder(buff, "Device_Control/Pack_DischargeFET"));
        mqttclient.subscribe(topicBuilder(buff, "Device_Control/Pack_ChargeFET"));
        mqttclient.subscribe(topicBuilder(buff, "Device_Control/Pack_SOC"));
        if (_settings.data.relaisFunction == 4)
          mqttclient.subscribe(topicBuilder(buff, "Device_Control/Relais"));
      }
      else
      {
        DEBUG_PRINT(F("Fail\n"));
      }
    }
    else
    {
      DEBUG_PRINT(F("Fail\n"));
      return false; // Exit if we couldnt connect to MQTT brooker
    }
    firstPublish = true;
  }
  return true;
}
