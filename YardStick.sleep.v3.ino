/** -*- C -*-
   @file

   @brief This code is a working soil & air sensor connected to the Losat cloud.
   It measures soil humidity plus air  temp & humidity.

   developped by Uri Bear 2018 for educational usage

   @note This programm is based on an example downloaded from Losant


*/

#define LOG_ERROR 0
#define LOG_WARNING 1
#define LOG_INFO 2
#define LOG_DEBUG 3
#define LOG_VERBOSE 4
//----------------------------------------------------//
// project definitions
#define PROJECT_NAME "YardStick"  // Used by provisioning mode as WiFi Access Point name
#define REPORT_INTERVAL 600       // in sec (theoretical)


// Infrastructure definitions
//------------------------------------------------------------------------------------//
// Start LOG functions

int verbose_lvl = LOG_INFO;  // Initial debug level for INFO style debug messages
//#define DEBUG_PROG         // master debug messages enable flag for DEBUG_PRINTLN style debug messages

char log_buffer[200];

#define LOG(lvl, format, ...) \
  do { \
    if(lvl <= verbose_lvl) {                                    \
      snprintf(log_buffer, sizeof(log_buffer), \
               format, ## __VA_ARGS__ ); \
      Serial.write(log_buffer);                                \
    }                                                           \
  }while(0)

#define ERR(format, ...) LOG(LOG_ERROR, format, ## __VA_ARGS__)
#define WARN(format, ...) LOG(LOG_WARNING, format, ## __VA_ARGS__)
#define INFO(format, ...) LOG(LOG_INFO, format, ## __VA_ARGS__)
#define DBG(format, ...) LOG(LOG_DEBUG, format, ## __VA_ARGS__)
#define VERBOSE(format, ...) LOG(LOG_VERBOSE, format, ## __VA_ARGS__)


#ifdef DEBUG_PROG
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)    Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

//----------------------------------------------------//
// Infrastructure libraries
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>     // used for configuration / provosioning
#include <DNSServer.h>            // used do divert web sessions to internal web server for configuration / provosioning
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <Losant.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <Ticker.h>               //for LED status
// #include <EasySSDP.h>             // http://ryandowning.net/EasySSDP/, currently not in use

//----------------------------------------------------//
// project specific libraries
#include "DHT.h"              // DHT22 is the air temp/humidity sensor

#define DHTTYPE DHT22   // DHT 22  (AM2302)



//-----------------
// Infrastructure pins
#define PROG_MODE_SEL_PIN     D5 // External switch, when waking up from sleep HIGH will cause an active wait & enable a terminal on serial port, use external PULLDOWN
#define SW_LED_PIN            D2 // LED pin, used for WiFi status
#define TRIGGER_PIN           D1 // Triggers the config portal when set to LOW. Use external PULLUP


// Project specific pins
#define ANALOG_PIN            A0 // Analog soil humidity sensor connection
#define SOIL_MEA_EN_PIN       D6 // Used to power up the soil humidity sensor

#define DHTPIN                D7 // Air humidity & temperature sensor connection
#define DHT_PWR_PIN           D8 // Used to power up the DHT22


//----------------------------------------------------//
// function declarations //
//-----------------
// Infrastructure
int Losant_setup();
char* printDouble( double val, unsigned int digits);
//void WiFI_reconnect();
void saveConfigCallback();
void tick();
void configModeCallback (WiFiManager *myWiFiManager);
// project specific infrastructure
void LosantUpdateData(float Air_temperature, float Air_Humidity, float Soil_Humidity);
void SerialPrintData(float Air_temperature, float Air_Humidity, float Soil_Humidity);

// project specific
float GetAnalogReading();
void GetTempHumid( float* Current_AirTemp, float* Current_AirHumid);
void GetSoilHumidity(float* Current_SoilHumidity);
void SaveAnalogPinParameters();
void LoadAnalogPinParameters();

//----------------------------------------------------//
// Sertial terminal handler functions definitions
typedef void (* GenericFP)(String); //function pointer prototype to a function which takes an 'String' an returns 'void', used by serial terminal
/** @struct _cmd
    @brief This structure structure that houses data for serial terminal command response entry
    @var _cmd::cmd
    Member 'cmd' contains the name of the command. This valu is compared against the command field in an incoming serial terminal request [Command, Arg0, Arg1 ....]
    @var _cmd::handler
    Member 'handler' contains a pointer to the function called when the incomming command matches the cmd. The handler is called with "Arg0, Arg1 ...." as it's argument
   @var _cmd::HelpDescriptor
    Member 'HelpDescriptor' contains a string that explains usage & syntax for thiscommand. This data is printed when the [help] terminal command is invoked.
*/
typedef struct _cmd
{
  String cmd;
  GenericFP handler;
  String HelpDescriptor;
} cmd_t;

int SerialStringBuilder(char* InputBuffer, int InputBufferSize, int NextPosition);
int CheckSerialBuffer(char* InputBuffer, int InputBufferSize, int NextPosition);
int ProcessInputBuffer(char* InputBuffer, int InputBufferSize, char* CommandBuffer, int CommandBufferSize, char* ArgumentBuffer, int ArgumentBufferSize);

// Handle serial port terminal requests
void yardstick_CommandHandler(String CommandString, String ArgumentsString);
void VERBOSE_handler (String InputArgs);
void parameter_erase_handler (String InputArgs);
void help_handler(String InputArgs);
void analog_read_handler(String InputArgs);
void DHT_ENmode_handler(String InputArgs);
void DHT_SetPowerOnDelay_handler(String InputArgs);

void handle_button_requests(int* GoDbg, int* WFMmode, char* InputBuffer, int InputBufferSize, int* InputBufferPointer, String* DebugMessage);
void LosantUpdateDebugMessage(String* DebugMessage);

//----------------------------------------------------//
// Losant default values.
//define your default values here, if there are different values in config.json, they are overwritten.
//-----------------
// Infrastructure
char Losant_device_ID[30] = ""; // place your default value here
char Losant_access_key[40] = ""; // place your default value here
char Losant_access_secret[70] = ""; // place your default value here

//----------------------------------------------------//
// Declare objects
//-----------------
// Infrastructure
WiFiClientSecure wifiClient; // Used by Losant
LosantDevice device("");     // Used by Losant
Ticker ticker;               // a timer for LED status

// project specific
DHT dht(DHTPIN, DHTTYPE);    // declate DHT object for air temp/humid measurements.

//----------------------------------------------------//
// Declare global variables & initial values
// Infrastructure
// int timeSinceLastRead = 0;
bool shouldSaveConfig = false; //flag for saving data

// project specific
int MAX_ANALOG_VAL = 397; // used as initial value, then automatically updates
int MIN_ANALOG_VAL = 258; // used as initial value, then automatically updates
int lastAnalogReading; // for analog soil humidity measurement

int DHT_DynamicPower = 0; // Selects if DHT powers up just before measurement (1) or always on (0)
int DHT_PowerOnDelay = 1000; // Sets the delay between DHT power up and first result request

//----------------------------------------------------//
/**
   @brief Setup tasks.

   This function  performs all tasks that are to be done once after device reset

   @note This is a required function under the Arduino IDE
   @note In this example we use DeepSleep between runs. ALL the devices functionality is done in THIS main function


*/
void setup()
{
  VERBOSE("VRB: setup:: Started.\n");

  // infrastructure variables
  char    InputBuffer[120];       // Used by serial port terminal
  int     InputBufferPointer = 0; // Used by serial port terminal
  int     GoDbg = 0;              // Flag, Enables serial port terminal mode
  int     WFMmode = 0;            // Flag, Indicates a request to re-enter the WiFiManager mode
  String  DebugMessage = "";      // Place to put debug strings to be sent to cloud.

  // project specific variables
  float   Air_Humidity = 0;       // Measurement result
  float   Air_temperature = 0;    // Measurement result
  float   Soil_Humidity = 0;      // Measurement result


  //----------------------------------------------------//
  Serial.begin(115200);
  DBG("DBG: setup:: %s debug terminal start...\n", PROJECT_NAME);
  //----------------------------------------------------//
  // Declare pins & pin mode
  // Infrastructure
  pinMode(TRIGGER_PIN, INPUT);        // trigger wifi connect mode
  pinMode(PROG_MODE_SEL_PIN, INPUT);  //
  pinMode(SW_LED_PIN, OUTPUT);        // set switch led pin as output
  digitalWrite(SW_LED_PIN, LOW);      // Turn off the LED

  // project specific
  pinMode(DHT_PWR_PIN, OUTPUT);       //
  pinMode(SOIL_MEA_EN_PIN, OUTPUT);   //
  digitalWrite(DHT_PWR_PIN, LOW);     // Turn off DHT
  digitalWrite(SOIL_MEA_EN_PIN, LOW); // Turn off soil measurement sensor

  //----------------------------------------------------//
  // Programm begin, infrastructure setup tasks
  if (DHT_DynamicPower == 0)
  {
    INFO("DBG: setup:: Powering on DHT sensor.\n");
    digitalWrite(DHT_PWR_PIN, HIGH);
  }
  handle_button_requests(&GoDbg, &WFMmode, InputBuffer, sizeof(InputBuffer), &InputBufferPointer, &DebugMessage); // Handle programming button and reconfig button
  LosantUpdateDebugMessage(&DebugMessage); // Send debug message to cloud. This is still under development

  int LosantConnectionResult = Losant_setup(); // Attempt connection to the Losant cloud, if cannot connect - do nothing.
  if (LosantConnectionResult > 0)
  {
    //----------------------------------------------------//
    DBG("Setup:: Starting project specific Setup.\n");
    dht.begin();                // start the DHT object
    LoadAnalogPinParameters();  // Load saved soil humidity sensor parameters if they exist
    VERBOSE("VRB: Setup:: Setup done.\n");
    //----------------------------------------------------//
    VERBOSE("VRB: Setup:: Starting the active(\"loop\") part.\n");

    GetTempHumid(&Air_temperature, &Air_Humidity);
    GetSoilHumidity(&Soil_Humidity);

    SerialPrintData(Air_temperature, Air_Humidity, Soil_Humidity); // Print data to serial
    LosantUpdateData(Air_temperature, Air_Humidity, Soil_Humidity);// Send data to cloud

    INFO("INF: setup:: Data sampled & sent to Losant!\n");
    device.disconnect();
  }
  else
    ERR("ERR: Setup:: Failed to connect to Losant, skipping active part.\n");
  //VERBOSE("VRB: setup:: ended, going to sleep for %d seconds.\n", REPORT_INTERVAL);
  INFO("INF: Setup:: Round completed, going to sleep for %d seconds.\n", REPORT_INTERVAL);
  Serial.flush(); // make sure serial print buffer was sent to serial port
  delay(2000); // To make sure serial port prints were completed before entering sleep

  ESP.deepSleep(REPORT_INTERVAL * 1e6); // deepSleep is defined in micro seconds
}

/**
   @brief continuous tasks.

   This function  performs all tasks that are to be run again and again once setup has completed

   @note This is a required function under the Arduino IDE
   @note This function is empty because this project uses DeepSleep

*/
void loop()
{
  // loop must exist for Arduino environment
} // end of main loop


//----------------------------------------------------//----------------------------------------------------//
// Project infrastructure related functions
/**
   @brief LosantUpdateData function.

   This function sends the required data to the Losant cloud service.

   @note Modify & expand this function as needed per project requirements

   @note This function uses JSON which is a way to format text to pass parameters to a remote server

   @param[in] Air_temperature - This is the air temperature in degrees Celsius as measured by the DHT22 sensor
   @param[in] Air_Humidity - This is the air humidity in percentage as measured by the DHT22 sensor
   @param[in] Soil_Humidity - This is the soil humidity sensor, measured by the ESP8266's Analog to Digital Converter (ADC) and converted to a percentage scale

*/
void LosantUpdateData(float Air_temperature, float Air_Humidity, float Soil_Humidity)
{
  VERBOSE("VRB: LosantUpdateData:: Started.\n");
  bool toReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    ERR("LosantUpdateData:: Disconnected from WiFi.\n");
    toReconnect = true;
  }

  if (!device.connected()) {
    ERR("LosantUpdateData:: Disconnected from MQTT. Err = %d.\n", device.mqttClient.state());
    toReconnect = true;
  }

  if (toReconnect) {
    //connect();
    //Losant_setup();

    int i = 0;
    while (i < 10)
    {
      INFO("connection attempt: %d\n", i);
      device.connectSecure(wifiClient, Losant_access_key, Losant_access_secret);
      int j = 0;
      while ((!device.connected()) && (j <= 10))
      {
        delay(500);
        INFO(".");
      }
      INFO("\n");
      if (device.connected())
        break;
    }
    if (!device.connected())
      ESP.restart();
    INFO("LosantUpdateData:: Connected to Losant!\n");
  }
  device.loop();

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["AirTemp"] = Air_temperature;
  root["AirHumid"] = Air_Humidity;
  root["SoilHumid"] = Soil_Humidity;
  DBG("LosantUpdateData:: Finished data build.\n");
  device.sendState(root);
  VERBOSE("VRB: LosantUpdateData:: END.\n");
}

/**
   @brief LosantUpdateDebugMessage function.

   This function sends the debug data string to the Losant cloud service.

   @param[in] DebugMessage - This is the pointer to the debug string to be sent.

*/
void LosantUpdateDebugMessage(String* DebugMessage)
{
  VERBOSE("VRB: LosantUpdateDebugMessage:: Started.\n");
  bool toReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    ERR("LosantUpdateDebugMessage:: Disconnected from WiFi.\n");
    toReconnect = true;
  }

  if (!device.connected()) {
    ERR("LosantUpdateDebugMessage:: Disconnected from MQTT. Err = %d.\n", device.mqttClient.state());
    toReconnect = true;
  }

  if (toReconnect)
  {
    //connect();
    //Losant_setup();

    int i = 0;
    while (i < 10)
    {
      INFO("connection attempt: %d\n", i);
      device.connectSecure(wifiClient, Losant_access_key, Losant_access_secret);
      int j = 0;
      while ((!device.connected()) && (j <= 10))
      {
        delay(500);
        INFO(".");
      }
      INFO("\n");
      if (device.connected())
        break;
    }
    if (!device.connected())
      ESP.restart();
    INFO("LosantUpdateData:: Connected to Losant!\n");
  }
  device.loop();

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["DebugMessage"] = &DebugMessage;
  device.sendState(root);
  VERBOSE("VRB: LosantUpdateDebugMessage:: END.\n");
}
void SerialPrintData(float Air_temperature, float Air_Humidity, float Soil_Humidity)
{
  VERBOSE("VRB: SerialPrintData:: Started.\n");

  DBG("Air temperature = %s [C].\n", printDouble(Air_temperature, 2));
  DBG("Air humidity = %s [%].\n", printDouble(Air_Humidity, 2));
  DBG("Soil humidity = %s [%].\n", printDouble(Soil_Humidity, 2));

  VERBOSE("VRB: SerialPrintData:: Ended.\n");
}
//----------------------------------------------------//
// Infrastructure related functions
/**
   @brief handle_button_requests function.

   This function handeles the reconfig pushbutton and serial console mode request switch

   @param[in] GoDbg - This is a pointer to a flag indicating the serial console mode request switch was pressed.
   @param[in] WFMmode - This is a pointer to a flag indicating the wifi reconfig request pushbutton was pressed.
   @param[in] InputBuffer - This is a pointer to a buffer that will contain the incoming command & arguments.
   @param[in] InputBufferSize - This is a the size of the InputBuffer.
   @param[in] InputBufferPointer - This is a pointer to the next available place in the InputBuffer. It is used to make the serial terminal noin-blocking
   @param[in] DebugMessage - This is a pointer to the DebugMessage string so debug data can be added to it if necessary, still under development

*/
void handle_button_requests(int* GoDbg, int* WFMmode, char* InputBuffer, int InputBufferSize, int* InputBufferPointer, String* DebugMessage)
{
  VERBOSE("VRB: handle_button_requests:: Started.\n");
  ticker.attach(0.2, tick);// start ticker with 0.2 - In terminal mode
  // Serial terminal variables
  char    CommandBuffer[20] = "";
  char    ArgumentBuffer[100] = "";
  *DebugMessage = *DebugMessage + String("HBR:: ");
  while (digitalRead(PROG_MODE_SEL_PIN) == HIGH)
  {
    if (*GoDbg == 0)
    {
      INFO("loop:: Waiting in terminal loop.\n");
      INFO("Waiting for command.\n");
      INFO("Command must look like [Command, Arg0 Arg1 ...][CR].\n");
      INFO(">");
      *DebugMessage = *DebugMessage + String("Waiting in terminal loop");
      *GoDbg = 1;
    }
    *InputBufferPointer = SerialStringBuilder(InputBuffer, InputBufferSize, *InputBufferPointer);
    if (*InputBufferPointer == -1)
    {
      ERR("main:: SerialStringBuilder has detected a Buffer overrun.\n");
      *InputBufferPointer = 0;
    }
    else
    {
      // check if the input string was terminated by a ']'
      int SerialPacketReady = CheckSerialBuffer(InputBuffer, InputBufferSize, *InputBufferPointer);
      //int SerialPacketReady = 1;
      if (SerialPacketReady > 0)
      {
        // Detected a well terminated string, go ahead and parse
        int InputStringReady = ProcessInputBuffer(InputBuffer, InputBufferSize, CommandBuffer, sizeof(CommandBuffer), ArgumentBuffer, sizeof(ArgumentBuffer));
        if (InputStringReady == 1) // Command is legal
        {
          DBG("DBG: TestCommandStructure:: Command buffer is \"%s\".\n", CommandBuffer);
          DBG("DBG: TestCommandStructure:: Argument buffer is \"%s\".\n", ArgumentBuffer);
          String CommandString(CommandBuffer);
          String ArgumentsString(ArgumentBuffer);
          yardstick_CommandHandler(CommandString, ArgumentsString);
        }
        // zero all buffers after execution
        InputBuffer[0] = 0;
        *InputBufferPointer = 0;
        CommandBuffer[0] = 0;
        ArgumentBuffer[0] = 0;

      }
    }
    delay(10);
    if (digitalRead(TRIGGER_PIN) == HIGH )  // is configuration portal requested?
    {
      INFO("WiFimanager trigger pressed, WFMmode = %d.\n>", *WFMmode);
      if (*WFMmode == 0)
      {
        INFO("Triggered into WiFimanager mode.\n>");
        *WFMmode = 1;
      }
      // WiFI_reconnect();
    }
  }
  ticker.detach(); // once WiFi is connected, stop blinking
  VERBOSE("VRB: handle_button_requests:: Ended.\n");
}

/**
   @brief Losant_setup function.

   This function opens a WiFi connection to the chosen network, then connects to the Losant cloud via MQTT

*/
int Losant_setup()
{
  INFO("INF: Losant_setup:: Losant_setup has started.\n");

  ticker.attach(0.6, tick);// start ticker with 0.5 because we start in AP mode and try to connect


  //
  //SPIFFS.format(); //clean FS, for testing
  //
  //read configuration from FS json
  DBG("DBG: Losant_setup:: mounting FS...\n");

  if (SPIFFS.begin())
  {
    DBG("DBG: Losant_setup:: mounted file system\n");
    if (SPIFFS.exists("/config.json")) // try to load the config file for Losant parameters
    {
      //file exists, reading and loading
      DBG("DBG: Losant_setup:: reading config file\n");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        DBG("DBG: Losant_setup:: opened config file for reading\n");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (verbose_lvl >= LOG_DEBUG)
          json.printTo(Serial);// XXX
        if (json.success())
        {
          DBG("DBG: Losant_setup:: parsed json");

          strcpy(Losant_device_ID, json["Losant_device_ID"]);
          strcpy(Losant_access_key, json["Losant_access_key"]);
          strcpy(Losant_access_secret, json["Losant_access_secret"]);

          DBG("DBG: Losant_setup:: Parameters are: Losant_device_ID = \"$s", Losant_device_ID);
          //DEBUG_PRINT(Losant_device_ID);
          DBG("\", Losant_access_key = \"%s", Losant_access_key);
          //DEBUG_PRINT(Losant_access_key);
          DBG("\", Losant_access_secret = \"%s\".\n", Losant_access_secret);
          //DEBUG_PRINT(Losant_access_secret);
          //DEBUG_PRINTLN("\".");

        }
        else
        {
          ERR("ERR: Losant_setup:: failed to load json config.\n");
        }
      }
    }
  }
  else
  {
    ERR("ERR: Losant_setup:: failed to mount FS.\n");
  }
  // Done with file read procedures
  //----------------------------------------------------------------------------------//
  // Adding parameters to the wifi configuration portal
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  //                   id/name placeholder/prompt default length
  WiFiManagerParameter custom_Losant_device_ID("ldid", "Losant device ID", Losant_device_ID, 30);
  WiFiManagerParameter custom_Losant_access_key("lak", "Losant access key", Losant_access_key, 40);
  WiFiManagerParameter custom_Losant_access_secret("las", "Losant access secret", Losant_access_secret, 70);

  WiFiManager wifiManager; //WiFiManager Local intialization. Once its business is done, there is no need to keep it around
  //reset settings - for testing
  //wifiManager.resetSettings();
  //
  wifiManager.setConfigPortalTimeout(300); // (in seconds) Give 5 minutes as timeout before webserver loop ends and exits even if there has been no setup.
  // usefull for devices that failed to connect at some point and got stuck in a webserver loop

  wifiManager.setConnectTimeout(60);  //(in seconds) Give 1 minute timeout for which to attempt connecting, usefull if you get a lot of failed connects



  wifiManager.setAPCallback(configModeCallback);//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setSaveConfigCallback(saveConfigCallback); //set config save notify callback

  //set static ip ??
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_Losant_device_ID);
  wifiManager.addParameter(&custom_Losant_access_key);
  wifiManager.addParameter(&custom_Losant_access_secret);

  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(PROJECT_NAME)) // fetches ssid and pass and tries to connect, if it does not connect it starts an access point with the specified name.
  {
    ERR("ERR: Losant_setup:: failed to connect to WiFi and hit timeout.\n");
    Serial.flush();
    ESP.reset(); //reset and try again, or maybe put it to deep sleep
    //delay(1000);
  }

  //if you get here you have connected to the WiFi
  IPAddress ip = WiFi.localIP();// XXX
  char ipCstring[16]; //size the char array large enough to hold 4 x 3 digit numbers + 3 x dot + null terminator
  utoa(ip[0], ipCstring, 10); //put the first octet in the array
  for (byte octetCounter = 1; octetCounter < 4; ++octetCounter) {
    strcat(ipCstring, ".");
    char octetCstring[4]; //size the array for 3 digit number + null terminator
    utoa(ip[octetCounter], octetCstring, 10);  //convert the octet to a string
    strcat(ipCstring, octetCstring);
  }
  INFO("INF: Losant_setup:: WiFi connected...yeey :) IP address: %s.\n", ipCstring);
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());

  ticker.detach(); // once WiFi is connected, stop blinking
  digitalWrite(BUILTIN_LED, LOW);//keep LED on

  //copy updated parameters from configuration page into global variables
  strcpy(Losant_device_ID, custom_Losant_device_ID.getValue());
  strcpy(Losant_access_key, custom_Losant_access_key.getValue());
  strcpy(Losant_access_secret, custom_Losant_access_secret.getValue());
  INFO("Losant_setup has finished, preparing to write to file.\n");

  if (shouldSaveConfig) //save the custom parameters to FS
  {
    DBG("DBG: Losant_setup:: saving config.\n");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["Losant_device_ID"] = Losant_device_ID;
    json["Losant_access_key"] = Losant_access_key;
    json["Losant_access_secret"] = Losant_access_secret;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      ERR("ERR: Losant_setup:: failed to open config file for writing.\n");
    }
    if (verbose_lvl >= LOG_DEBUG)
      json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
  } //end save

  //LosantDevice device(LOSANT_DEVICE_ID);
  device.setId(Losant_device_ID);
  INFO("INF: Losant_setup:: Attempting connection to Losant.\n");
  DBG("DBG: Losant_setup:: Losant_device_ID: \"%s\", Losant_access_key: \"%s\", Losant_access_secret: \"%s\".\n", Losant_device_ID, Losant_access_key, Losant_access_secret);
  device.connectSecure(wifiClient, Losant_access_key, Losant_access_secret);
  int LosantConnectionAttampts = 0;
  int LosantConnectionAttamptsLimit = 50;
  while ((!device.connected()) && (LosantConnectionAttampts < LosantConnectionAttamptsLimit))
  {
    delay(500);
    if ((LosantConnectionAttampts % 10) == 0)
      INFO("\n");
    else
      INFO(".");
    LosantConnectionAttampts = LosantConnectionAttampts + 1;
  }
  if (device.connected())
  {
    INFO("INF: Losant_setup:: Connected to Losant!\n");
    return 1;
  }
  else
  {
    ERR("ERR: Losant_setup:: Failed to connect to Losant!\n");
    return -1;
  }
  //EasySSDP::begin(server, PROJECT_NAME);

  //INFO("The Losant device is now ready for use!\n");

}

/**
*   @brief printDouble function.
*
*   This function converts a Double to a char buffer for printing to the serial port, used by the debug messages. It prints val with number of decimal places determine by precision
*
*   @note Maximum string length limited to 16 chars
*   @note use %s to print the result of this code
*   @note Example: float SoilMeasurementResult = MesureData(); Info("This is the meaurement result: %s.\n", printDouble(SoilMeasurementResult));
*
*   @param[in] val - float type value to be printed
*   @param[in] digits - The number of digits requested AFTER the decimal point
*
*   @note Eexample: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)
*
*/
char* printDouble( double val, unsigned int digits)
{
  // prints val with number of decimal places determine by precision
  long          precision = pow(10, digits);
  int           index = 0;
  static  char  s[16];
  char          format[16];
  unsigned int  frac;
  if (val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val) - val ) * precision;
  sprintf(format, "%%d.%%0%dd", digits);
  sprintf(s, format, int(val), frac);

  return s;
}

/**
*   @brief WiFI_reconnect function.
*
*   This function reconnects to the WiFi network. To be used if a disconnect was reported.
*
*/
void WiFI_reconnect()
{
  VERBOSE("VRB: WiFI_reconnect:: Started.\n");
  bool toReconnect = false;
  WiFiManager wifiManager; //Local intialization. Once its business is done, there is no need to keep it around
  /* ---------------------------------------------*/
  // Some options for debugging
  //wifiManager.resetSettings(); //reset settings - for testing

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration

  //WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.1
  //WiFi.mode(WIFI_STA);
  /* ---------------------------------------------*/

  if (!wifiManager.startConfigPortal(PROJECT_NAME))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  if (!device.connected())
  {
    Serial.println("Disconnected from MQTT");
    Serial.println(device.mqttClient.state());
    toReconnect = true;
  }
  if (toReconnect)
  {
    Serial.print("Connecting to Losant...");
    //device.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
    device.connectSecure(wifiClient, Losant_access_key, Losant_access_secret);

    while (!device.connected())
    {
      delay(500);
      Serial.print(".");
    }
  }
  VERBOSE("VRB: WiFI_reconnect:: Ended.\n");
}


//
/**
*   @brief saveConfigCallback function.
*
*  This function is a callback notifying us of the need to save config
*
*/
void saveConfigCallback ()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/**
*  @brief tick function.
*
*   This function is a callback used to toggle the LED pin state (Blink the LED)
*
*/

void tick()
{
  //toggle state
  int state = digitalRead(SW_LED_PIN);  // get the current state of GPIO1 pin
  digitalWrite(SW_LED_PIN, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
/**
*   @brief configModeCallback function.
*
*   This function gets called when a request to re-enable the WiFi config mode is detected
*
*   @param[in] myWiFiManager - a pointet to the myWiFiManager object
*
*/
void configModeCallback (WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}
//----------------------------------------------------//
// Project specific functions

/**
*   @brief SaveAnalogPinParameters function.
*
*   This function saves the analog in minimum and maximum values to the flash
*   @note These values are used to convert the analog pin raw measurement to a percentage
*
*
*/
void SaveAnalogPinParameters()
{
  // Use this function to save the analog pin min/max parameters
  VERBOSE("VRB: SaveAnalogPinParameters:: Started.\n");
  char MinValueStr[10];
  char MaxValueStr[10];
  itoa(MIN_ANALOG_VAL, MinValueStr, 10);
  itoa(MAX_ANALOG_VAL, MaxValueStr, 10);
  DEBUG_PRINTLN("saving soil sensor config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["SoilSensor_MinValue"] = MinValueStr;
  json["SoilSensor_MaxValue"] = MaxValueStr;

  File configFile = SPIFFS.open("/AnalogPin_config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }
  if (verbose_lvl >= LOG_DEBUG)
    json.printTo(Serial);
  Serial.println("");
  json.printTo(configFile);
  configFile.close();
  VERBOSE("VRB: SaveAnalogPinParameters:: Ended.\n");
}

/**
*   @brief LoadAnalogPinParameters function.
*
*   This function loads the analog in minimum and maximum values to the flash
*   @note These values are used to convert the analog pin raw measurement to a percentage
*
*
*/
void LoadAnalogPinParameters()
{
  // Use this function to load the saved parameters - if they exist
  // Otherwise use defaults

  VERBOSE("VRB: LoadAnalogPinParameters:: Started.\n");

  char AnalogPin_MinValueStr[10] = "0.00";
  char AnalogPin_MaxValueStr[10] = "100.00";

  if (SPIFFS.begin())
  {
    DBG("DBG: LoadAnalogPinParameters:: mounted file system.\n");
    if (SPIFFS.exists("/AnalogPin_config.json"))
    {
      //file exists, reading and loading
      DBG("DBG: LoadAnalogPinParameters:: reading Soil sensor config file.\n");
      File configFile = SPIFFS.open("/AnalogPin_config.json", "r");
      if (configFile)
      {
        DBG("DBG: LoadAnalogPinParameters:: opened SoilSensor config file\n");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (verbose_lvl >= LOG_DEBUG)
          json.printTo(Serial);
        if (json.success())
        {
          DBG("\nparsed json\n");

          strcpy(AnalogPin_MinValueStr, json["SoilSensor_MinValue"]);
          strcpy(AnalogPin_MaxValueStr, json["SoilSensor_MaxValue"]);
          MAX_ANALOG_VAL = atoi(AnalogPin_MaxValueStr);
          MIN_ANALOG_VAL = atoi(AnalogPin_MinValueStr);

          DBG("DBG: LoadAnalogPinParameters:: Loaded MAX_ANALOG_VAL from file (%d).\n", MAX_ANALOG_VAL);
          DBG("DBG: LoadAnalogPinParameters:: Loaded MIN_ANALOG_VAL from file (%d).\n", MIN_ANALOG_VAL);

        }
        else
        {
          ERR("ERR: LoadAnalogPinParameters:: failed to load SoilSensor json config file.\n");
        }
      }
    }
  }
  VERBOSE("VRB: LoadAnalogPinParameters:: Ended.\n");
}

/**
*   @brief GetAnalogReading function.
*
*   This function reads the raw analog pin conversion result, then cunverts it to a percentage value
*
*   @note The value is converted  to percentage using previously saved minimum and maximum values.
*   @note The minimum and maximum values are updated if new read value is outside these limits.
*
*   @return Analog value in percentage
*
*/
float GetAnalogReading()
{
  // Read analog pin, filter and percentalize
  // lastAnalogReading is a global persistant value for filtering

  VERBOSE("GetAnalogReading:: started.\n");
  int   CurrentAnalogReading = analogRead(ANALOG_PIN);
  DBG("Analog value: %d.\n", CurrentAnalogReading);
  // filter
  //lastAnalogReading += (CurrentAnalogReading - lastAnalogReading) / 10;
  lastAnalogReading = CurrentAnalogReading; // used by a filter when DeepSleep was not used
  DBG("Filtered analog value: %d.\n", lastAnalogReading);
  if (CurrentAnalogReading < MIN_ANALOG_VAL)
  {
    MIN_ANALOG_VAL = CurrentAnalogReading;
    //lastAnalogReading = CurrentAnalogReading;
    DBG("Soil humidity MIN_ANALOG_VAL adjusted to: %s.\n", printDouble(CurrentAnalogReading, 2));
    SaveAnalogPinParameters();
  }
  if (CurrentAnalogReading > MAX_ANALOG_VAL)
  {
    MAX_ANALOG_VAL = CurrentAnalogReading;
    //lastAnalogReading = CurrentAnalogReading;
    DBG("Soil humidity MAX_ANALOG_VAL adjusted to: %s,\n", printDouble(CurrentAnalogReading, 2));
    SaveAnalogPinParameters();
  }
  // calculate soil humidity in %
  float CurrentAnalogValuePercent = map(lastAnalogReading, MIN_ANALOG_VAL, MAX_ANALOG_VAL, 0, 100); // map(value, fromLow, fromHigh, toLow, toHigh)
  DBG("Percentilized analog value is: %s.\n", printDouble(CurrentAnalogValuePercent, 2));
  VERBOSE("GetAnalogReading:: ended.\n");
  return CurrentAnalogValuePercent;
}

/**
*   @brief GetSoilHumidity function.
*
*   This function turns on the soil humidity sensor and, after a delay, does the measurement and returns the result.
*
*   @param[out] Current_SoilHumidity - This is a pointer to the result
*/
void GetSoilHumidity(float* Current_SoilHumidity)
{
  digitalWrite(SOIL_MEA_EN_PIN, HIGH);// Turn on soil measurement sensor
  delay(200); // Power up to data valid delay
  *Current_SoilHumidity    = GetAnalogReading();
  digitalWrite(SOIL_MEA_EN_PIN, LOW);// Turn off soil measurement sensor
}

/**
*   @brief GetTempHumid function.
*
*   This function turns on the air temp & humidity sensor and, after a delay, does the measurement and returns the results.
*
*   @param[out] Current_AirTemp - This is a pointer to the air temperature result
*   @param[out] Current_AirHumid - This is a pointer to the air humidity result
*/
void GetTempHumid( float* Current_AirTemp, float* Current_AirHumid)
{
  // Get DHT sensor data
  VERBOSE("GetTempHumid:: Started.\n");

  if (DHT_DynamicPower == 1)
  {
    DBG("DBG: GetTempHumid:: Powering off DHT sensor.\n");
    //digitalWrite(DHT_PWR_PIN, HIGH);
    delay(DHT_PowerOnDelay); // Power up to data valid delay
  }

  for (int i = 0; i < 5; i++)
  {
    *Current_AirTemp             = dht.readTemperature(); // dht.getTemperature();
    *Current_AirHumid            = dht.readHumidity(); // dht.getHumidity();
    if (isnan(*Current_AirTemp) || isnan(*Current_AirHumid))
    {
      ERR("GetTempHumid:: Failed to read from DHT sensor. i=%d!\n", i);
      delay(100);
    }
    else
    {
      DBG("GetTempHumid:: DHT22 Temp=%s", printDouble(*Current_AirTemp, 2));
      DBG(", DHT22 Humidity=%s.\n", printDouble(*Current_AirHumid, 2));
      break;
    }
  }
  if (DHT_DynamicPower == 1)
  {
    DBG("DBG: GetTempHumid:: Powering off DHT sensor.\n");
    //digitalWrite(DHT_PWR_PIN, LOW);
  }
  VERBOSE("GetTempHumid:: Ended.\n");
}

//---------------------------------------------------------------//
// The following section handeles the serial console
/**
*   @brief SerialStringBuilder function.
*
*   This function builds the input buffer each time a new character arrives. It is written as a non-blocking function
*
*   @param[in] InputBuffer - This is a pointer to the buffer
*   @param[in] InputBufferSize - This is the input buffer size
*   @param[in] NextPosition - This is the next place to write to in the input buffer.
*/
int SerialStringBuilder(char* InputBuffer, int InputBufferSize, int NextPosition)
{
  /// This function is called when data is available in the serial buffer.
  /// it builds the serial string until the string end character is received.
  bool PrintDbgMessage = false;

  VERBOSE("SerialStringBuilder:: Started.\n");

  while ((Serial.available()) &&  (NextPosition < InputBufferSize))// Data is available at Serial
  {
    char inChar = Serial.read(); // Read data from serial
    InputBuffer[NextPosition] = inChar;
    Serial.write(inChar);
    NextPosition++;
    PrintDbgMessage = true;
  }
  // check end conditions
  if (NextPosition == InputBufferSize)
  {
    ERR("SerialStringBuilder:: Buffer overrun\n");
    return -1;
  }
  else
  {
    VERBOSE("SerialStringBuilder:: Ended.\n");
    if (PrintDbgMessage)
      DBG("SerialStringBuilder:: Next position is %d.\n", NextPosition);
    return NextPosition;
  }
}

/**
*   @brief CheckSerialBuffer function.
*
*   This function checks if the data available in the serial port input buffer is ready.
*   it checks for two conditions:
*        Current character (last character in buffer) is a [CR] (\n) - then it terminates the string buffer with a \0 and returns the buffer length
*        if buffer is not in ready, it returns -1;.
*
*   @param[in] InputBuffer - This is a pointer to the buffer
*   @param[in] InputBufferSize - This is the input buffer size
*   @param[in] NextPosition - This is the next place to write to in the input buffer.
*/
int CheckSerialBuffer(char* InputBuffer, int InputBufferSize, int NextPosition)
{
  // This function checks if the data available in the serial port input buffer is ready.
  // it checks for two conditions:
  // Current character (last character in buffer) is a [CR] (\n) - then it terminates the string buffer with a \0 and returns the buffer length
  // if buffer is not in ready, it returns -1;.
  VERBOSE("CheckSerialBuffer:: Started.\n");
  if (InputBuffer[NextPosition - 1] == ']')
  {
    InputBuffer[NextPosition] = 0;
    VERBOSE("VRB: CheckSerialBuffer:: Ended, a well terminater string was found.\n");
    INFO("\n");
    return NextPosition;
  }
  else
  {
    VERBOSE("CheckSerialBuffer:: Ended, string not terminated yet.NextPosition = %d.\n", NextPosition);
    return -1;
  }
}

/**
*   @brief ProcessInputBuffer function.
*
*   This function processes the input buffer.
*   it checks for proper structure, then seperates the command from the arguments and strips whitespaces.
*
*   @param[in] InputBuffer - This is a pointer to the buffer
*   @param[in] InputBufferSize - This is the input buffer size
*   @param[out] CommandBuffer - This is a pointer to the buffer that will contain the Command part of the buffer, after trimming
*   @param[in] CommandBufferSize - This is the Command buffer size
*   @param[out] ArgumentBuffer - This is a pointer to the buffer that will contain the Arguments part of the buffer, after trimming
*   @param[in] ArgumentBufferSize - This is the Arguments buffer size
*
*   @return 0 if not ready, 1 if command seperation OK .... Need to complete
*/
int ProcessInputBuffer(char* InputBuffer, int InputBufferSize, char* CommandBuffer, int CommandBufferSize, char* ArgumentBuffer, int ArgumentBufferSize)
{
  // This function is called when a well terminated string was received in the serial port.
  // The function now searches if a command string can be found in the input string
  // a command string is defined as as a string that starts with a "[" character and is terminated with a "]" character.
  // if a valid command string is found, it is stripped of the limiting characters, trimmed of null characters and passed to be decoded.
  // the rest of the string is trimmed and sent as the argument string

  VERBOSE("ProcessInputBuffer:: Started");

  int i = 0;
  int LegalBufferStart;
  int LegalBufferEnd;
  char TempString[120];
  String CommandString;
  String ArgumentsString;
  char TempCmdBuff[20];
  char TempArgBuff[100];



  while ((InputBuffer[i] != ']') && (i < InputBufferSize))// Data is available for analysis, look for end character
  {
    i++;
  }
  if (InputBuffer[i] == ']')
  {
    LegalBufferEnd = i;
    DBG("ProcessInputBuffer:: Found buffer end at position %d.\n", i);
    i = 0;
    while ((InputBuffer[i] != '[') && (i < LegalBufferEnd))// look for buffer start
    {
      i++;
    }
    if (InputBuffer[i] == '[')
    {
      //INFO("Legal string is \"%s\"\n", )
      LegalBufferStart = i + 1;
      DBG("ProcessInputBuffer:: Found buffer start at position %d and end at position %d.\n", LegalBufferStart, LegalBufferEnd);
      i = 0;
      for (int j = LegalBufferStart; j < LegalBufferEnd; j++)
      {
        TempString[i] = InputBuffer[j];
        i++;
      }
      TempString[i] = 0;
      //String CommandString = inString.substring(StringStart+1, StringEnd);
      DBG("ProcessInputBuffer:: Command sub-string is \"%s\"\n", TempString);
      String CommandString(TempString);

      CommandString.trim();
      DBG("ProcessCommandLine:: Trimmed Command sub-string is \"%s\"\n", CommandString.c_str());

      if (CommandString.indexOf(',') > 0)
      {
        CommandString = strtok(TempString, ",");
        //DBG("ProcessInputBuffer:: Command buffer is \"%s\"\n", CommandString.c_str());
        //String CommandBufferString(CommandBuffer);
        //INFO("ProcessInputBuffer:: Command String is \"%s\"\n", CommandBufferString.c_str());
        CommandString.trim();
        //INFO("ProcessInputBuffer:: Trimmed Command String is \"%s\"\n", CommandString.c_str());

        CommandString.toCharArray(TempCmdBuff, sizeof(TempCmdBuff));
        int CmdStrLen = CommandString.length();
        strncpy(CommandBuffer, TempCmdBuff, CmdStrLen);
        CommandBuffer[CmdStrLen] = 0;
        //INFO("ProcessInputBuffer:: Trimmed Command buffer is \"%s\"\n", CommandBuffer);

        ArgumentsString = strtok(NULL, ",");
        //INFO("ProcessInputBuffer::Argument buffer is \"%s\"\n", ArgumentsString.c_str());
        //String ArgumentBufferString(ArgumentBuffer);
        //INFO("ProcessInputBuffer:: Argument String is \"%s\"\n", ArgumentBufferString.c_str());
        ArgumentsString.trim();
        //INFO("ProcessInputBuffer:: Trimmed Argument String is \"%s\"\n", ArgumentsString.c_str());
        ArgumentsString.toCharArray(TempArgBuff, sizeof(TempArgBuff));
        int ArgStrLen = ArgumentsString.length();
        strncpy(ArgumentBuffer, TempArgBuff, ArgStrLen);
        ArgumentBuffer[ArgStrLen] = 0;
      }
      else
      {
        DBG("ProcessInputBuffer:: Command only state\n");
        CommandString.toCharArray(TempCmdBuff, sizeof(TempCmdBuff));
        int CmdStrLen = CommandString.length();
        strncpy(CommandBuffer, TempCmdBuff, CmdStrLen);
        CommandBuffer[CmdStrLen] = 0;
        ArgumentBuffer[0] = 0;

      }
      DBG("ProcessInputBuffer:: Command is \"%s\", Arguments are \"%s\"\n", CommandBuffer, ArgumentBuffer);
      VERBOSE("ProcessInputBuffer:: Ended");
      return 1;
    }
    else
    {
      ERR("ProcessInputBuffer:: Buffer Start character is missing\n");
    }
  }
  else if (i == InputBufferSize)//InputBufferSize
  {
    ERR("ProcessInputBuffer:: Buffer overrun. i=%d, InputBufferSize = %d, size of input buffer = %d\n", i , InputBufferSize, sizeof(InputBuffer));
    VERBOSE("ProcessInputBuffer:: Ended");
    return -1; // Buffer overrun
  }
  else
  {
    VERBOSE("ProcessInputBuffer:: Ended");
    return 0;
  }
}




//void DHT_ENmode_handler(String InputArgs), void DHT_SetPowerOnDelay_handler(String InputArgs)
cmd_t cmds[] = {
  {"z", parameter_erase_handler, "[z]: Erase analog pin parameters"},
  {"h" , help_handler, "[h]: Print help for all available commands"},
  {"a", analog_read_handler, "[a]: Measure & display raw analog pin values"},
  {"DDP", DHT_ENmode_handler, "[DDP, B]: Enable DHT sensor dynamic power control (B = 1, default) or always on (B = 0)"},
  {"DDPD", DHT_SetPowerOnDelay_handler, "[DDPD, X]: Set DHT sensor dynamic power delay (PWR_En to first measurement) to X mSec's"},
  //  {"VERBOSE", VERBOSE_handler},
  //  {"GET_IP_ADDR", GET_IP_ADDR_handler},
  //  {"GET_IP_ADDR", GET_IP_ADDR_handler},
  //  {"GET_IP_ADDR", GET_IP_ADDR_handler},
  //  {"GET_IP_ADDR", GET_IP_ADDR_handler},
  {"v", VERBOSE_handler, "[v, X]: Set verbose level to X. 0 = LOG_ERROR, 1 = LOG_WARNING, 2 = LOG_INFO, 3 = LOG_DEBUG, 4 = LOG_VERBOSE."}
  //  {"VERBOSE", VERBOSE_handler, "[VERBOSE, X]: Set verbose level to X. 0 = LOG_ERROR, 1 = LOG_WARNING, 2 = LOG_INFO, 3 = LOG_DEBUG, 4 = LOG_VERBOSE."}
};

/**
*   @brief yardstick_CommandHandler function.
*
*   This function processes the Command buffer.
*   it checks for a match between the Command string and any of the Command strings in the cmds array.
*   If a match was found, then the proper handler is called with the ArgumentsString as it's argument.
*
*   @param[in] CommandString - This is a String object that contains the Command to be matched
*   @param[in] ArgumentsString - This is a String object that contains the Arguments to the matched handler
*
*/
void yardstick_CommandHandler(String CommandString, String ArgumentsString)
{
  VERBOSE("yardstick_CommandHandler:: Started.\n");
  DBG("DBG: yardstick_CommandHandler:: command = \"%s\",Arguments = \"%s\"\n", CommandString.c_str(), ArgumentsString.c_str());
  const char* String1 = CommandString.c_str();

  for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
  {

    const char* String2 = cmds[i].cmd.c_str();
    VERBOSE("VRB: yardstick_CommandHandler:: Comparing command = \"%s\",to constant = \"%s\"\n", String1, String2);
    if (strcmp(String1, String2) == 0)
    {
      DBG("DBG: yardstick_CommandHandler::  Match found\n");
      cmds[i].handler(ArgumentsString);
      DBG("DBG: yardstick_CommandHandler::  handler executed\n");
    }
  }
  VERBOSE("yardstick_CommandHandler:: Ended.\n");
}

/**
*   @brief VERBOSE_handler function.
*
*   This function is a handler to a specific Command.
*   it changes the verbose level for the entire project per Arg0
*
*   @param[in] InputArgs - This is a String object that contains the Arguments to the this handler
*
*/
void VERBOSE_handler (String InputArgs)
{

  VERBOSE("VRB: VERBOSE_handler:: Started");
  DBG("DBG: VERBOSE_handler:: InputArgs = \"%s\".\n", InputArgs.c_str());
  verbose_lvl = atoi(InputArgs.c_str());
  INFO("levels:: 0 = LOG_ERROR, 1 = LOG_WARNING, 2 = LOG_INFO, 3 = LOG_DEBUG, 4 = LOG_VERBOSE.\n");
  INFO("Verbose level set to %d.\n", verbose_lvl);
  INFO("\n>"); // Prompt for the next command
  VERBOSE("VRB: VERBOSE_handler:: Ended");
}

/**
*   @brief parameter_erase_handler function.
*
*   This function is a handler to a specific Command.
*   it sets the analog pin limits to the absolutely WRONG values, so that the next analog reads will set the proper bounds
*   These bounds are used to convert the raw analog read to a percentage value.
*   The modified values are saved to the flash file for future runs.
*
*   @param[in] InputArgs - No arguments are used by this handler
*
*/
void parameter_erase_handler (String InputArgs)
{
  VERBOSE("VRB: parameter_erase_handler:: Started.\n");
  INFO("parameter_erase_handler:: Erasing analog parameters.\n");
  MIN_ANALOG_VAL = 1024;
  MAX_ANALOG_VAL = 0;
  SaveAnalogPinParameters();
  INFO("\n>"); // Prompt for the next command
  VERBOSE("VRB: parameter_erase_handler:: Ended.\n");
}

/**
*   @brief help_handler function.
*
*   This function is a handler to a specific Command.
*   it prints out the help section per command from the cmds array
*
*   @param[in] InputArgs - No arguments are used by this handler
*
*/
void help_handler(String InputArgs)
{
  VERBOSE("VRB: help_handler:: Started.\n");
  int NumOfCmds = sizeof(cmds) / sizeof(cmd_t);
  DBG("DBG: help_handler:: NumOfCmds = %d, sizeof(cmds) = %d, sizeof(cmd_t) = %d.\n", NumOfCmds, sizeof(cmds), sizeof(cmd_t));
  for (int i = 0; i < NumOfCmds; i++)
  {
    INFO("%s\n", cmds[i].HelpDescriptor.c_str());
  }
  INFO(">"); // Prompt for the next command
  VERBOSE("VRB: help_handler:: Ended.\n");
}

/**
*   @brief analog_read_handler function.
*
*   This function is a handler to a specific Command.
*   it cdoes an analog read and returns the raw value to the terminal, along the saved MIN & MAX values that were read from the flash file.
*
*   @param[in] InputArgs - No arguments are used by this handler
*
*/
void analog_read_handler(String InputArgs)
{
  VERBOSE("VRB: analog_read_handler:: Started.\n");
  int   CurrentAnalogReading = analogRead(ANALOG_PIN);
  INFO("loop:: Analog read RAW value is %d [Analog] [%d...%d].\n", CurrentAnalogReading, MIN_ANALOG_VAL, MAX_ANALOG_VAL);
  INFO("\n>"); // Prompt for the next command
  VERBOSE("VRB: analog_read_handler:: Ended.\n");
}

/**
*   @brief DHT_ENmode_handler function.
*
*   This function is a handler to a specific Command.
*   it changes the DHT22 power mode from always enabled (1) to power-just-for-measurement mode (0).
*
*   @param[in] InputArgs - Arg0 is the value to be assigned to the DHT_DynamicPower variable.
*
*/
void DHT_ENmode_handler(String InputArgs)
{
  VERBOSE("VRB: DHT_ENmode_handler:: Started.\n");
  DBG("DBG: DHT_ENmode_handler:: InputArgs = \"%s\".\n", InputArgs.c_str());
  DHT_DynamicPower = atoi(InputArgs.c_str());
  INFO("DBG: DHT_ENmode_handler:: DHT_DynamicPower set to %d.\n", DHT_DynamicPower); //
  INFO("\n>"); // Prompt for the next command
  VERBOSE("VRB: DHT_ENmode_handler:: Ended.\n");
}

/**
*   @brief DHT_SetPowerOnDelay_handler function.
*
*   This function is a handler to a specific Command.
*   it changes the DHT_PowerOnDelay per Arg0. This value is the delay from dynamic power application to measurement in mSecs.
*
*   @param[in] InputArgs - This is a String object that contains the Arguments to the this handler
*
*/
void DHT_SetPowerOnDelay_handler(String InputArgs)
{
  VERBOSE("VRB: DHT_SetPowerOnDelay_handler:: Started.\n");
  DBG("DBG: DHT_SetPowerOnDelay_handler:: InputArgs = \"%s\".\n", InputArgs.c_str());
  DHT_PowerOnDelay = atoi(InputArgs.c_str());
  INFO("DBG: DHT_SetPowerOnDelay_handler:: DHT_PowerOnDelay set to %d.\n", DHT_PowerOnDelay);
  INFO("\n>"); // Prompt for the next command
  VERBOSE("VRB: DHT_SetPowerOnDelay_handler:: Ended.\n");
}


