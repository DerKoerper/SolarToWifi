/*
DALY BMS to MQTT Project
https://github.com/softwarecrash/DALY-BMS-to-MQTT
This code is free for use without any waranty.
when copy code or reuse make a note where the codes comes from.
*/

#include <Arduino.h>

class Settings
{
  // change eeprom config version ONLY when new parameter is added and need reset the parameter
  unsigned int configVersion = 10;

public:
  struct Data
  {                             // do not re-sort this struct
    unsigned int coVers;        // config version, if changed, previus config will erased
    char deviceName[40];        // device name
    char mqttServer[40];        // mqtt Server adress
    char mqttUser[40];          // mqtt Username
    char mqttPassword[40];      // mqtt Password
    char mqttTopic[40];         // mqtt publish topic
    unsigned int mqttPort;      // mqtt port
    unsigned int mqttRefresh;   // mqtt refresh time
    bool mqttJson;              // switch between classic mqtt and json
    bool wakeupEnable;  // use wakeup output?
    bool relaisFailsafe; // relais failsafe mode | false - turn off, true - keep last state
    bool relaisEnable;  // enable relais output?
    bool relaisInvert;  // invert relais output?
    byte relaisFunction;    // function mode - 0 = Lowest Cell Voltage, 1 = Highest Cell Voltage, 2 = Pack Cell Voltage, 3 = Temperature
    byte relaisComparsion;  // comparsion mode - 0 = Higher or equal than, 1 = Lower or equal than
    float relaisSetValue; // value to compare to !!RENAME TO SOMETHING BETTER!!
    float relaisHysteresis; // value to compare to
  } data;

/*  void load()
  {
    data = {}; // clear bevor load data
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, data);
    EEPROM.end();
    coVersCheck();
    sanitycheck();
  }

  void save()
  {
    sanitycheck();
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, data);
    EEPROM.commit();
    EEPROM.end();
  }

  void reset()
  {
    data = {};
    save();
  }
*/

private:
  // check the variables from eeprom

  void load_settings()
  {
      strcpy(data.deviceName, "SolarToWifi");
      strcpy(data.mqttServer, "192.168.0.50");
      strcpy(data.mqttUser, "");
      strcpy(data.mqttPassword, "");
      strcpy(data.mqttTopic, "akkukiste");
      data.mqttPort = 1883;
      data.mqttRefresh = 300;
      data.mqttJson = false;
      data.wakeupEnable = true;
      data.relaisFailsafe = false;
      data.relaisEnable = true;
      data.relaisInvert = false;
      data.relaisFunction = 0;
      data.relaisComparsion = 0;
      data.relaisSetValue = 0.0;
      data.relaisHysteresis = 0.0;
  }
};
