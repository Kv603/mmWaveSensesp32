//#define LITELED
#ifdef LITELED
#define DECAYINTERVAL 4
// Brightness level (0-255, where 0=off, 255=full brightness)
#define LED_BRIGHTNESS 7
uint8_t StripBrightness = LED_BRIGHTNESS;
#endif

#include "secrets.h"
#include <ctype.h>
#include <Preferences.h>  // NVS preferences  library
#define PREF_RO true
#define PREF_RW false

// Needed for WifiClient among other calls
#include <WiFi.h>

// Where do we send debug output?
#define DEBUG_ESP_PORT Serial
//#define DEBUG_ASYNC_MQTT_CLIENT3


#define HOURS_BETWEEN_FORCED_REBOOTS 48

const char LottaDashes[] PROGMEM = "---------------------------------";
#define NOTYETDISCONNECTED "Not yet attempted"

// Interval between MQTT retries, in seconds
#define MQTTRECONINTERVAL 31

#define TOPICBUFFERSIZE 255
const String StringNever  = "Never";
const String StringUndef  = "Undef";
const char Topic_Influx_Occupancy[] PROGMEM = "timeseries/occupancy";
const char Topic_Floorplan[] PROGMEM = "floorplan";
const char Topic_HomeSeer[] PROGMEM = "sensor";
const char Topic_Broadcast[] PROGMEM = "msg/broadcast";
const char BodySignString[] = "bodysign";
const char PresenceString[] = "motion";  // Movement for MQTT for homeseer
const char NoBodySignWarningMessage[] = "Cannot post, no valid BodySign yet received";
const char MQTTBroadcastTSubFailed[] = "MQTT subscription to msg/broadcast failure!";
const char ConnSucc[] = "Successful Connect.";
const char NotNeededString[] = "unneccessary";
const char FormURLEncoded[] PROGMEM = "application/x-www-form-urlencoded";
// Debugging Serial port baud rate
//#define BAUD 9600
#define BAUD 115200

#include <stdarg.h>

// Should we just do a full reboot if we have a critical failure reading the I2C bus/devices?
// #define REBOOT_ON_IIC_FAILURE true

// What board is this?
#ifdef NEOPIXEL_POWER_ON
#undef ESP32POE
#ifndef ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM
#define ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM
#endif
#endif
boolean UseLedStrip = false;


#ifndef LITELED
#define LED_COUNT 0
#else
#define LED_COUNT 16
size_t LEDCount = LED_COUNT;
#include <LiteLED.h>
#define LED_PIN 5

#define BASE_LED_VALUE_HIGHWATER LED_COUNT
LiteLED strip(LED_STRIP_WS2812, false);
void updateLedMeter(LiteLED *strip, unsigned int currentValue, unsigned int numPixels = LED_COUNT);
#endif
void DisplayBodySign();

uint8_t DecayBodySign(boolean noop = false) __attribute__((always_inline));
// Using the board found in "C:\Users\passp\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.9\variants\esp32-poe"

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12
#include <ETH.h>
#include "esp_mac.h"


const char TextHtml[] PROGMEM = "text/html";
const char TextPlain[] PROGMEM = "text/plain";
const char ContentType[] PROGMEM = "Content-Type";

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
// Create client objects as global
WiFiClient *CleartextClient = new WiFiClient;
WiFiClientSecure *SecureClient = new WiFiClientSecure;
WiFiClientSecure *client = new WiFiClientSecure;

// Used for non-storage (NVS) small data
Preferences SavedNVS;
//#define NEED_DNS
//#ifdef NEED_DNS
//#include "lwip/dns.h"
//#endif


// XIAO C3 is single core.
#ifndef APP_CPU
#ifdef XIAO_ESP32C3
#define APP_CPU 0
#else
#define APP_CPU 1
#endif
#endif

#ifdef XIAO_ESP32C3
#include "Arduino.h"
#endif
//For OVER_THE_AIR_FIRMWARE
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <ESPAsyncWebSrv.h>

#ifdef NEWELEGANTOTA
#include <ElegantOTA.h>
void onOTAStart();
void onOTAProgress(size_t current, size_t final);
void onOTAEnd(bool success);
#endif

void setDNSpublic();
void StartHumanStaticPresenceLite();
void handleRadarMode(boolean force = false);
void SetRadarMode();
void ResetRadar();
void publish_bodysign_MQTT(int value, boolean force = false);
void publish_bodysign_Influx_MQTT(int value, boolean force = false);
int post_Detail_iotplotter(int status, const char *key = NULL, int value = 0);
boolean FloorplanBodySign(int, boolean);

void BlankRetainedRadarMQTT();
inline int RadarStatus() __attribute__((always_inline));

// Easy SNTP client library from Arduino-core
// We might as well also bring in DateTime too
#include <NTPClient.h>
// WiFiudp is required for NTPClient?
#include <WiFiUdp.h>
const char NTPSERVERNAME[] PROGMEM = "us.pool.ntp.org";

#include <DateTime.h>
#include <DateTimeTZ.h>


// Used by healthcheck etc
#include <uptime.h>
#include <uptime_formatter.h>

// buffer for serial receive and get UID
#define BUFFER_SIZE 64
#define MSGBUFFER_SIZE 65
#define URLBUFSIZE 256
#define ERRBUFSIZE 212
#define POSTBUFSIZE 1024
// We allocate JSONDOC dynamically from the heap
#define JSONDOCSIZE 4096

#ifdef DOHEALTHCHECK
#include <ArduinoJson.h>
char LastHealthCheckResult[URLBUFSIZE] = "None yet";
#else
const char LastHealthCheckResult[] = "Compiled w/o HC";
#endif

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <esp_system.h>
#include <esp32-hal-log.h>
#include <esp_task_wdt.h>  // For watchdog timeout, also requires Arduino.h


// include an MQTT server
#include <Ticker.h>
#include <AsyncMqtt_Generic.h>



#define DEFAULTLASTWEBERRORSTRING "No saved error"
TaskHandle_t WatchedTask = NULL;
// How long (in seconds) can we hang before the watchdog forces a reboot?
#define WDT_TIMEOUT 607
boolean MustFeedWatchDog = false;
//esp_err_t init_watchdog(uint32_t timeout, bool panic);
inline void feed_watchdog() __attribute__((always_inline));

inline void do_delay(unsigned int ms) __attribute__((always_inline));
inline unsigned long GetUpMinutes() __attribute__((always_inline));


//  If using charlieplexed LEDs on GPIO 18 (A0) and 17 (A1)
// See https://learn.adafruit.com/adafruit-qt-py-esp32-s3/pinouts


////////////////////////// End board specific definitions
#define SECONDS_BETWEEN_IOTPLOTTER_CALLS 29
#define SECONDS_BETWEEN_SETTING_CLOCK 1000
#define SECONDS_BETWEEN_MQTT_BODYSIGNPUBLISHES 23

#define SECONDS_BETWEEN_FLOORPLAN_PUBLISHES 17

#define SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES 47
#define FORCEDRADARIOTPLOTTERINTERVAL 4 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES
double ForcedRadarIotPlotterInterval = FORCEDRADARIOTPLOTTERINTERVAL;

#define UPTIMEPUBLISHINTERVAL 900
double UptimePublishInterval = UPTIMEPUBLISHINTERVAL;
time_t LastUptimePublished = 0;

time_t LastPresencePublishTime = 0;
int LastPresencePublishValue = -1;

#define FORCEDRADARNOMESSAGEPUBLISHINTERVAL 600
double ForcedRadarNoMessagePublishInterval = FORCEDRADARNOMESSAGEPUBLISHINTERVAL;

#define SECONDS_BETWEEN_INFLUXMQTT_PUBLISHES 333
double InfluxMQTTPublishInterval = SECONDS_BETWEEN_INFLUXMQTT_PUBLISHES;

#define SECONDS_BETWEEN_MQTT_PUBLISHES 139
double MQTTPublishInterval = SECONDS_BETWEEN_MQTT_PUBLISHES;

#define RADARSETMODEINTERVAL 39 * 60
double RadarSetModeInterval = RADARSETMODEINTERVAL;
#define RADARRESETINTERVAL 1201
double RadarResetInterval = RADARRESETINTERVAL;

#define WIFICHECKINTERVAL 600
double WiFiCheckInterval = WIFICHECKINTERVAL;
#define ONEDAYINTERVAL 86400
#define THREEHOURINTERVAL (60 * 60 * 3)
//#define POSTHEAPINTERVAL 3599
#define POSTHEAPINTERVAL 900
double PostHeapInterval = POSTHEAPINTERVAL;

#define HEALTHCHECKINTERVAL 1800
double HealthCheckInterval = HEALTHCHECKINTERVAL;

// Flag indicating we have not ever successfully posted
#define NEVERPOSTED 0
#define INVALIDBODYSIGN -1138
#define PURGEBODYSIGN -42

// Handshake timeout is in seconds
#define HTTP_HANDSHAKE_TIMEOUT 12000
// How long to wait before we just give up?
#define HTTP_TIMEOUT_DATA 21000
#define HTTP_CONNECT_TIMEOUT 18000



#include <TimeElapsed.h>

#ifdef USE_MDNS
#include <ESPmDNS.h>
#endif

// For MatchState
//#include <Regexp.h>

#ifndef CONFIG_FREERTOS_NUMBER_OF_CORES
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1
#endif


#ifdef NEOPIXEL_POWER_ON
#define MDNS_NAME "QtPymmWave32s3"
#else
#define MDNS_NAME "ESP32mmWave2"
#endif

#ifdef ESP32POE
#undef MDNS_NAME
#define MDNS_NAME "mmWave32poe"
#endif

#ifdef LITELED
#define VERSION "0.72mmLED"
#else
#define VERSION "0.72mm"
#endif

const char TIMEZONE[] PROGMEM = "TZ_America_New_York";
const char TZ[] PROGMEM = "EST+5EDT,M3.2.0/2,M11.1.0/2";
const char zeroip[] = "0.0.0.0";

// For preferences NVS
const char nvs_MQTTServer[] = "MQTTServer";
const char nvs_whereword[] = "Where";
const char nvs_Floor[] = "Floor";
const char nvs_Room[] = "Room";
const char nvs_DisableSensor[] = "DisableSensor";
const char nvs_healthurlpause[] = "healthurlp";
const char nvs_healthmodurl[] = "healthurlm";
const char nvs_healthpingurl[] = "healthurl";
const char nvs_healthresumeurl[] = "healthurlr";
const char nvs_LastWebError[] = "LWE";
const char nvs_LEDCount[] = "LEDs";

#define SLEEPDURATION 1500
unsigned int SleepDuration = SLEEPDURATION;

#define MAX_WIFI_RETRIES 50
#define WIFI_LOST 5000
#define ETHERNET 8710
#define RSSI_UNKNOWN -32768

#define BUFFER_ONELINE 16

void onMqttMessage(const char *topic, const char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total);

// Can go as low as 2000 for 2s intervals
#define MQTT_CHECK_INTERVAL_MS 28000

void BigLoop(void *arg);

void force_interval_reboot() __attribute__((always_inline));
void PostHeapIOTPlotter();
inline void WarnDisabled() __attribute__((always_inline));

#if (!PLATFORMIO)
// Enable Arduino-ESP32 logging in Arduino IDE
#ifndef CORE_DEBUG_LEVEL
#define CORE_DEBUG_LEVEL 4
#endif
#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
#endif
#endif

void updateModes(boolean bootcall = false);
void readNVS();
inline void writeNVS() __attribute__((always_inline));
void writeLastWebErrorToNVS(int errorcode);

#define HTTP_HEADER_USER_AGENT VERSION
boolean startNTP(boolean);
boolean ConnectNetwork();
void StartEthernet();
time_t getNow();

//For MQTT
void onMqttDisconnect(AsyncMqttClientDisconnectReason);

void logram(void);

char *getMyIP(boolean forcedhcp = false);
const char *shortID(boolean force = false);
String humanReadableTime(time_t timestamp);
boolean NetworkGood(boolean usecached = true);
char *FindMyIP();
inline boolean goodClock() __attribute__((always_inline));
esp_reset_reason_t logbootreason(void);

boolean publishError(const char *message, const time_t when = 0);

String humanReadableSize(const size_t bytes);
//size_t getArduinoLoopTaskStackSize(void);
boolean FloorplanBodySign(int val, boolean force = false);
void DoHumanStaticPresenceLite();

// Mostly useful for healthchecks
char *makeSlug(const char *input, char *output);

#ifdef DOHEALTHCHECK
// HealthChecks.IO
int RemoteHealthAPI(const char *longurl);
inline boolean RemoteHealthCallModify() __attribute__((always_inline));
inline int handleDoHealthCheck() __attribute__((always_inline));
const char *RemoteHealthCallRegister(boolean force = false);
int RemoteHealthCall(int healthy, const char *status, const char *agentstring = NULL);
#endif

void display_RSSI(boolean serial = false, long rssi = RSSI_UNKNOWN);
char *RemoveCharacter(char unwanted, char *str);
const char *uniqueID(boolean force = false);
const char *hostName();  // Like Unique ID, but doesn't change when mode changes

char NodeID[BUFFER_SIZE];
char NodeIDshort[20] = "Pre-Init";

void handleStats(AsyncWebServerRequest *request);

// For OVER_THE_AIR_FIRMWARE
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
void setupwebserver(const char *user, const char *pass);
#ifndef HTTP_USER
#define HTTP_USER "REPLACEME"
#define HTTP_PASS "WITHSOMETHINGBETTER"
#endif



AsyncMqttClient mqttClient;


// Variables which might be modified in a function or task are volatiles
bool connectedMQTT = false;

bool WiFiBegan = false;

bool ProcessingOTA = false;
int EthernetConnected = 0;

#ifdef ESP32POE
boolean WantWiFi = false;
#else
boolean WantWiFi = true;  // Do we want to fall back to WiFi if necessary?
#endif

bool UpdatedModes = false;


bool LastWebRequestSucceeded = true;
bool WaitingForClock = true;
int LastWebRequestCode = 0;
int LastPlotterRequestCode = 0;

unsigned int NetworkFailureCount = 0;
time_t lastClockSet = 0;
time_t LastWiFiCheckTime = 0;
time_t LastPostHeapTimeSeconds = 0;
time_t LastBodySignFloorplanTimeSeconds = 0;

time_t LastFedWatchdog = 0;

time_t TimeNow = 0;  // In seconds
boolean ForcePublish = true;
boolean ForcePlot = false;
////////////////
/// Globals
unsigned long UpMinutes = 0;
unsigned long RadarMessageCount = 0;

time_t LastDetailPostTimeSeconds = 0;
time_t LastBodySignPostTimeSeconds = 0;
time_t LastStatusPostTimeSeconds = 0;
time_t LastStatusPostSuccessTimeSeconds = 0;
time_t LastBodySignPostSuccessTimeSeconds = 0;
time_t LastDetailPostSuccessTimeSeconds = 0;
time_t BodySignValCaptureTime = 0;
time_t LastRadarModeSetTime = 0;                  // initialize
unsigned long NextIOTPlotterPostTimeSeconds = 0;  // When should we next post to IOTPlotter?
unsigned long NextIOTPlotterHeapPostSeconds = 0;  // Same, but for the heap stats
int StatusValueLastPosted = NEVERPOSTED;
int LastPostedBodySignVal = INVALIDBODYSIGN;
int LastFloorplanBodySignVal = INVALIDBODYSIGN;

int BodySignPeakVal = 0;

int BodySignVal = INVALIDBODYSIGN;
char TimeString[BUFFER_SIZE] = "0000-00-00";
char UptimeString[BUFFER_SIZE] = "Up 00000";
char MyIPAddress[32] = "000.000.000.000";
char MQTTserver[BUFFER_SIZE] = MQTT_SERVER;
char MQTTserverAddress[BUFFER_SIZE] = "Unknown";
Ticker connectToMqttTicker;
Ticker mqttReconnectTimer;

time_t LastBodySignMQTTTimeSeconds = 0;
int BodySignlastMQTT = INVALIDBODYSIGN;

int BodySignlastInfluxMQTT = INVALIDBODYSIGN;
time_t LastBodySignInfluxMQTTTimeSeconds = 0;

char MQTTReasonText[TOPICBUFFERSIZE * 2] = NOTYETDISCONNECTED;

char MQTTreport[URLBUFSIZE];
char MQTTtopic[TOPICBUFFERSIZE];

char IotPlotterResponseString[ERRBUFSIZE] = "uninit";

time_t LastRadarMessageHandledTime = 0;
time_t LastHealthCheck = 0;
time_t LastHealthCheckSuccess = 0;
time_t LastLedUpdate = 0;

RTC_NOINIT_ATTR unsigned int BodySignBaseVal;
RTC_NOINIT_ATTR long long ForceInitStaticMemory;  // If non-zero, we will assume static memory must be re-init'd.
RTC_NOINIT_ATTR time_t LastRadarMessageTime;      // Absolute time of last message we saw
RTC_NOINIT_ATTR unsigned long LastIOTPlotterAttemptTime;

char RadarMessage[MSGBUFFER_SIZE] = "Nothing received yet";
char Where[BUFFER_SIZE] = TAG;
char OldName[BUFFER_SIZE] = "";
char Floor[BUFFER_SIZE] = "";
char DisableSensor[BUFFER_SIZE] = "";
boolean DisabledSensor = false;
char Room[BUFFER_SIZE] = "";
std::string NoWheres;

#ifdef DOHEALTHCHECK
char HealthCheckPingURL[URLBUFSIZE] = "";
char HealthCheckPauseURL[URLBUFSIZE] = "";
char HealthCheckResumeURL[URLBUFSIZE] = "";
char HealthCheckUpdateURL[URLBUFSIZE] = "";
#endif
 
char LastWebError[URLBUFSIZE] = DEFAULTLASTWEBERRORSTRING;      // Last HTTP error we saw, if any
char LastPlotterError[URLBUFSIZE] = DEFAULTLASTWEBERRORSTRING;  // Last HTTP error we saw, if any
char BootReasonText[MSGBUFFER_SIZE] = "Reason Unknown";
int LastHealthCheckCode = -42;  //  Last result code from a healthcheck.

// Shared array to conserve stack
char PostDataBuffer[POSTBUFSIZE] = "uninit";
char RamDataBuffer[POSTBUFSIZE] = "uninit";
char IotPlotterDataBuffer[POSTBUFSIZE] = "uninit";

#ifdef DOHEALTHCHECK
char BigJSONbuffer[POSTBUFSIZE] = "Uninit";
#endif

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
<TITLE>%UNAME% %DISABLED% %VERSION%: RADAR %FLOOR%/%ROOM%/%WHERE%</TITLE></HEAD>
<BODY><H1>ESP32 Radar %UNAME%<BR>%DISABLED%</H1><BR>
<CENTER><H2 ALIGN=CENTER>%FLOOR%/%ROOM%/%WHERE%</H2></CENTER><BR>
<P>Successfully posted BodySign value %LASTBODYSIGNPOSTVAL% %LASTBODYSIGNPOSTSUCCESS%, successfully posted status %LASTSTATUSPOSTSUCCESS%, posted detail message at %LASTDETAILPOSTSUCCESS%.</P>
<UL>
<LI><A HREF="/config">Configure Device</A></LI> 
<LI><A HREF="/stats">Print statistics</A> or <A HREF="/watch">Watch readings</A></LI> 
<LI><A HREF="/iotplotter">Show IOTplotter activity</A></LI>
<LI><A HREF="/mqtt">Print MQTT status</A> or <A HREF="/mqttserver">change server name</A></LI> 
<LI><A HREF="/update">Update via OTA</A> (or just <A HREF="/forcerestart">force a reboot</A></LI>
</UL>
<HR>
<P>Uptime %UPMINUTES% minutes, Last Fed Watchdog %LASTFEDWATCHDOG%, %FEEDINGWATCHDOG%, must be fed by %WATCHEDTASK% every %FEEDWATCHDOG%.</P>
<P>We have processed %RADARMESSAGECOUNT% radar events, peak bodysign (since reboot) is %BODYSIGNPEAK%, nadir is %BODYSIGNBASE%<BR>
 Last Radar message arrived at %LASTRADARTIME% was "%RADARMESSAGE%" as of %RADARREADTIME%<BR>
 Firmware version is Radar %VERSION% %BUILD% on an %USB_MANUFACTURER% '%USB_PRODUCT%'.</P>
<UL> 
</body></html>
)rawliteral";



const char mqttserverform_html[] PROGMEM = R"rawliteral(
 <!DOCTYPE HTML><html lang="en"></head><body %BCOLOR%>
<H1>Set MQTT Server</H1>
<P><A HREF="/">Go Home</A><BR></P>
<P>Sent %MESSESMQTT% messages to MQTT Server '%MQTTSERVER%'.  Enter a new MQTT Server here to be saved to non-volatile storage (NVS)</P><B>
<form action="/mqttserver" method="post">
<input type="text" name="mqttserver" maxlength="31" required><button type="submit">Submit</button></form>

<form action="/mqttserver" method="post">
<input type="hidden" name="mqttserver" value="delete"><button type="submit">Clear Value</button></form>
<BR><P>Or you can just <A HREF="/">go home</A>.</P>
</body></html>
)rawliteral";


const char iotplotter_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %DISABLED% %DISABLE% (%FLOOR%/%ROOM%/%WHERE%) %UNAME% %VERSION% Statistics</title>
  <meta http-equiv="refresh" content="9" /><meta http-equiv="pragma" content="no-cache"/>
  <HTML><H1><A HREF="/">%UNAME%</A> IOTplotter status as of %DATETIME% %DISABLED%</H1><BR>
  <CENTER><H2 ALIGN=CENTER>Radar %FLOOR%/%ROOM%/%WHERE%</H2></CENTER><BR>
<P>Last BodySign IOTPlotter radar posting of %LASTBODYSIGNPOSTVAL% was %LASTBODYSIGNPOST%,
  detail message was posted %LASTDETAILPOSTTIME%, we sent "%IOTPLOTTERDATA%"</P>
<BR><P>You can view at <A HREF="https://iotplotter.com/user/feed/176009403279807659">IOTPlotter</A>, or you can just <A HREF="/">go home</A>.</P>
</body></html>
)rawliteral";


const char stats_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %DISABLED% %DISABLE% (%FLOOR%/%ROOM%/%WHERE%) %UNAME% %VERSION% Statistics</title>
  <meta http-equiv="refresh" content="9" /><meta http-equiv="pragma" content="no-cache"/>
  <HTML><H1><A HREF="/">%UNAME%</A> status as of %DATETIME% %DISABLED%</H1><BR>
  <CENTER><H2 ALIGN=CENTER>Radar %FLOOR%/%ROOM%/%WHERE%</H2></CENTER>
  <P>Uptime %UPMINUTES% minutes, Last Fed Watchdog %LASTFEDWATCHDOG%, %FEEDINGWATCHDOG%, must be fed by %WATCHEDTASK% every %FEEDWATCHDOG%.
 processed %RADARMESSAGECOUNT% radar events, peak bodysign has been %BODYSIGNPEAK%, nadir is %BODYSIGNBASE%, 
  last Radar message %LASTRADARMSG% (at %LASTRADARTIME%) <STRONG>"%RADARMESSAGE%"</STRONG>
  as of %RADARREADTIME%</P>
<PRE>Last web request result was %WEBCODE% %WEBERROR%: %HTTPERROR% <BR>
Last Plotter request result was %PLOTTERCODE% %PLOTTERERROR%<BR>
Last BodySign <A HREF="/iotplotter">IOTPlotter</A> radar posting of %LASTBODYSIGNPOSTVAL% was %LASTBODYSIGNPOST%,
Detail message was posted %LASTDETAILPOSTTIME%, we sent "%IOTPLOTTERDATA%"<BR>
Successfully posted BodySign %LASTBODYSIGNPOSTSUCCESS%, status %LASTSTATUSPOSTSUCCESS%, detail at %LASTDETAILPOSTSUCCESS%.<BR>
Posted %LASTFLOORPLANVAL% to Floorplan at %LASTFLOORPLANTIME%.<BR>
Have seen %NETWORKFAILURECOUNT% network connection drops.<BR>
%DISABLE%  if disabled, string is "%DISABLE%"<BR>
</PRE>
<P><A HREF="/runfast">Click here</A> to cut all time interval waits in half (dangerous)</P>
<HR><P>Uptime:  (%UPMINUTES% raw minutes), system start reason was %BOOTREASON%</P>
<P>Memory Statistics<UL>
<LI>Flash Size: %SIZEFLASH%</LI>
<LI>Total Heap: %TOTALHEAP%</LI>
<LI>Free Heap: %FREEHEAP% with largest fragment being %FRAGHEAP%</LI>
<LI>Total Stack: %TOTALSTACK%</LI>
<LI>Free Stack: %FREESTACK%</LI>
</UL></p>
<P>Clock Status<UL>
<LI>NTP Status:  %NTPSTATUS%, Server: %NTPSERVER%, %NTPTIME%</LI> 
<LI>DateTime: %DATETIME%</LI>
<LI>BootTime: %BOOTTIME%, system start reason %BOOTREASON%/LI>
</UL>
<P>HomeSeer</P>
<UL> 
<LI>Floor: %FLOOR%</LI>
<LI>Room: %ROOM%</LI>
</UL>

<P>Device Status</P>
<UL>
<LI>Firmware version is  %VERSION%
<LI>%QUIET%   %DISABLED% Disable string is '%DISABLE%'</LI>  
<LI><A HREF="/rhc">Healthcheck</A> returned %HEALTHLASTCHECKCODE% at %HEALTHLASTCHECK%</LI>
<LI>MQTT server is %MQTTSERVER% (%MQTTSERVERADDRESS%), topic is "%MQTTTOPIC%", connection is %MQTTCONNECTED% and 
  last MQTT disconnect reason was %MQTTREASON%</LI> 
<LI>Firmware version is Radar %VERSION% %BUILD% on an %USB_MANUFACTURER% '%USB_PRODUCT%'.</LI>
</UL>
</p>
<P>We sent IotPlotter: %IOTPLOTTERSENT%, IOTPlotter returned: %IOTPLOTTERANSWER%<BR></P>
<P>Uptime %UPMINUTES% minutes, Last Fed Watchdog %LASTFEDWATCHDOG%, %FEEDINGWATCHDOG%, must be fed by %WATCHEDTASK% every %FEEDWATCHDOG%.</P>
<BR><HR><P><A HREF="/">Go back home</A></P>
</BODY></HTML>
)rawliteral";


const char watch_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %BODYSIGN% last seen  %LASTRADARMSG%</title>
  <meta http-equiv="refresh" content="3" /><meta http-equiv="pragma" content="no-cache"/>
  <HTML><H1><A HREF="/">%UNAME%</A> As of  %LASTRADARMSG%, got %BODYSIGN%  max %BODYSIGNPEAK%, nadir is %BODYSIGNBASE%</H1><BR>
<UL><LI>Uptime %UPMINUTES% minutes, system start reason was %BOOTREASON%<BR> Last Fed Watchdog %LASTFEDWATCHDOG%, %FEEDINGWATCHDOG%, must be fed every %FEEDWATCHDOG%.</LI>
<LI>Processed %RADARMESSAGECOUNT%  events, latest was %RADARMESSAGE% at %LASTRADARTIME%</LI></UL>
<P>We sent: %IOTPLOTTERSENT%<BR>
IOTPlotter returned: %IOTPLOTTERANSWER%<BR></P>
<P>Memory Statistics<UL>
<LI>Flash Size: %SIZEFLASH%</LI>
<LI>Total Heap: %TOTALHEAP%</LI>
<LI>Free Heap: %FREEHEAP% with largest fragment being %FRAGHEAP%</LI>
<LI>Total Stack: %TOTALSTACK%</LI>
<LI>Free Stack: %FREESTACK%</LI>
</UL></p><BR><HR><P><A HREF="/">Go back home</A></P>
</BODY></HTML>
)rawliteral";


#ifdef DOHEALTHCHECK
const char health_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html lang="en"><head>
<meta name="viewport" content="width=device-width, initial-scale=1"><meta charset="UTF-8">
<title>Radar %UNAME% %VERSION% Health Check Info</title> 
<HTML><H1><A HREF="/">%UNAME%</A> Health Check Info as of %DATETIME%</H1>
<P>Last attempted health check was %HEALTHLASTCHECK%, last result was %HEALTHLASTCHECKCODE% "%HEALTHLASTCHECKRESULT%".  Last successful check-in was %HEALTHLASTCHECKSUCCESS%</P>
<P> You can force this device to <A HREF="/rhcregister">Re-register</A>.</P><BR>
<P>Health Check  details are at  <A HREF="https://healthchecks.io/projects/16e57218-138a-42e4-96df-789901ee13b5/checks/">Healthchecks.IO</A>.</P>
<P>You can <A HREF="/rhcping">force a ping</A>.</P>
<P>Last web request result was %WEBCODE% %WEBERROR%: %HTTPERROR%  (Saved NVS web error was %SAVEDWEBERROR%)</P>
<P>If we did a health-check registration, buffer returned was "%BIGJSONBUFFER%".</P>
<P>Health Check URLs<UL>
<LI>Ping %HEALTHCHECKURL%</LI>
<LI>Pause %HEALTHCHECKPAUSEURL%</LI>
<LI>Resume %HEALTHCHECKRESUMEURL%</LI>
<LI>Modify %HEALTHCHECKUPDATEURL%</LI></UL>
<P>last Radar message  at %LASTRADARTIME% was "%RADARMESSAGE%", we have processed %RADARMESSAGECOUNT% radar events, and last got radar data %RADARREADTIME%</LI></P>
<P>We sent: %IOTPLOTTERSENT%<BR>
IOTPlotter returned: %IOTPLOTTERANSWER%<BR></P>
<HR>
<P>Uptime:  (%UPMINUTES% raw minutes),  Last Fed Watchdog %LASTFEDWATCHDOG%, %FEEDINGWATCHDOG%, must be fed by %WATCHEDTASK% every %FEEDWATCHDOG%.</P>
<P>System start reason was %BOOTREASON%</P>
<P>Memory Statistics<UL>
<LI>Flash Size: %SIZEFLASH%</LI>
<LI>Total Heap: %TOTALHEAP%</LI>
<LI>Free Heap: %FREEHEAP% with largest fragment being %FRAGHEAP%</LI>
<LI>Total Stack  %TOTALSTACK%</LI>
<LI>Free Stack  %FREESTACK%</LI>
</UL></p>
<P>Device Status</P>
<UL>
<LI>Firmware version is  %VERSION%
<LI>Compiled %BUILD%</LI> 
<LI>Hardware is %USB_MANUFACTURER% '%USB_PRODUCT%</LI>
</UL>
</p><BR><HR><P><A HREF="/">Go back home</A></P>
</BODY></HTML>
)rawliteral";


const char health_txt[] PROGMEM = R"rawliteral( 
<P>We sent: %IOTPLOTTERSENT%<BR>
IOTPlotter returned: %IOTPLOTTERANSWER%<BR></P>
<PRE>Last Web %WEBCODE% %WEBERROR%: %HTTPERROR% 
Memory - Free / Total 
Heap -  %FREEHEAP%/%TOTALHEAP%  (largest fragment being %FRAGHEAP%)
Stack - %FREESTACK% / %TOTALSTACK%  
NTP is %NTPSTATUS%, RTC is at %RTCADDRESS% and status is %RTCSTATUS%, Uptime %UPTIME%, reboot cause %BOOTREASON%
Firmware version is  %VERSION% %BUILD% on an %USB_MANUFACTURER% '%USB_PRODUCT%'.
</PRE>
<BR><HR><P><A HREF="/">Go back home</A></P>
)rawliteral";
#endif

const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %UNAME% %VERSION% Settings</title> 
  <HTML><H1>Settings for %UNAME%</H1>
 
<form action="/configure" method="post">
  <label for="text">Name:
  Device Name <input type="text" id="name" name="name" value="%WHERE%"></label><br>
  Floor <input type="text" id="%FLOORNVS%" name="%FLOORNVS%" value="%FLOOR%"></label><br>
  Room <input type="text" id="%ROOMNVS%" name="%ROOMNVS%" value="%ROOM%"></label><br>
   Disable:<input type="text" id="%NVSDISABLE%" name="%NVSDISABLE%" value="%DISABLE%"></label> (Remove value or type enable in field to re-enable)<br>
   <input type="submit" value="Submit">
</form>
<BR><HR><P><A HREF="/">Go back home</A></P>
</p>
</BODY></HTML>
)rawliteral";


////////////////////////
const char mqtt_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %DISABLED% (%FLOOR%/%ROOM%/%WHERE%) %UNAME% %VERSION% MQTT Statistics</title>
  <meta http-equiv="refresh" content="9" /><meta http-equiv="pragma" content="no-cache"/>
  <HTML><H1><A HREF="/">%UNAME%</A> MQTT status as of %DATETIME%</H1><BR>
  <CENTER><H2 ALIGN=CENTER>Radar %FLOOR%/%ROOM%/%WHERE%</H2></CENTER><BR>
   
<LI> <A HREF="/mqttgraph">Graph live stream</A></LI>
 
<LI>We last published to MQTT %LASTBODYSIGNMQTTTIME%,  
we last received a radar event at %LASTRADARTIME%, 
and have processed %RADARMESSAGECOUNT% radar events,last Radar message was "%RADARMESSAGE%",
we last got radar data  %LASTRADARMSG% timestamp %RADARREADTIME%</LI>
<LI><A HREF="/mqttserver">MQTT server is %MQTTSERVER%</A>, topic is "%MQTTTOPIC%", connection is %MQTTCONNECTED% and 
  last MQTT disconnect reason was %MQTTREASON%</LI> 
<LI>Posted %LASTFLOORPLANVAL% to Floorplan at %LASTFLOORPLANTIME%.</LI>
 <LI>Our IotPlotter prefix is %NOWHERE%.
</p><BR><HR><P><A HREF="/">Go back home</A></P>
</BODY></HTML>
)rawliteral";


// <script src="https://code.highcharts.com/highcharts.js"></script>
#ifdef USE_GRAPHER
const char mqttgraph_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="en">
<head><script src="https://cam.pinesec.org/lowcharts/highcharts.js"></script>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <title>Radar %DISABLED% (%FLOOR%/%ROOM%/%WHERE%) %UNAME% %VERSION% MQTT Statistics</title> 
 <script type="text/javascript" src="http://ajax.googleapis.com/ajax/libs/jquery/1.8.2/jquery.min.js"></script>
<script src="mqttws31.js" type="text/javascript"></script>
<script type="text/javascript">
 

//settings BEGIN
	var MQTTbroker = '%MQTTSERVERADDRESS%';
	var MQTTport = %MQTTPORTWS%;
  var datapoints = 60;
	var MQTTsubTopic = 'sensor/#'; //works with wildcard # and + topics dynamically now
//settings END

	var chart; // global variable for chart
	var dataTopics = new Array();

//mqtt broker 
	var client = new Paho.MQTT.Client(MQTTbroker, MQTTport,
				"myclientid_" + parseInt(Math.random() * 100, 10));
	client.onMessageArrived = onMessageArrived;
	client.onConnectionLost = onConnectionLost;
	//connect to broker is at the bottom of the init() function !!!!
	

//mqtt connecton options including the mqtt broker subscriptions
	var options = {
		timeout: 3,
		onSuccess: function () {
			console.log("mqtt connected");
			// Connection succeeded; subscribe to our topics
			client.subscribe(MQTTsubTopic, {qos: 1});
		},
		onFailure: function (message) {
			console.log("Connection failed, ERROR: " + message.errorMessage);
			//window.setTimeout(location.reload(),20000); //wait 20seconds before trying to connect again.
		}
	};

//can be used to reconnect on connection lost
	function onConnectionLost(responseObject) {
		console.log("connection lost: " + responseObject.errorMessage);
		//window.setTimeout(location.reload(),20000); //wait 20seconds before trying to connect again.
	};

//what is done when a message arrives from the broker
	function onMessageArrived(message) {
		console.log(message.destinationName, '',message.payloadString);

		//check if it is a new topic, if not add it to the array
		if (dataTopics.indexOf(message.destinationName) < 0){
		    
		    dataTopics.push(message.destinationName); //add new topic to array
		    var y = dataTopics.indexOf(message.destinationName); //get the index no
		    
		    //create new data series for the chart
			var newseries = {
		            id: y,
		            name: message.destinationName,
		            data: []
		            };

			chart.addSeries(newseries); //add the series
		    
		    };
		 
		var y = dataTopics.indexOf(message.destinationName); //get the index no of the topic from the array
		var myEpoch = new Date().getTime(); //get current epoch time
		var thenum = message.payloadString.replace( /^\D+/g, ''); //remove any text spaces from the message
		var plotMqtt = [myEpoch, Number(thenum)]; //create the array
		if (isNumber(thenum)) { //check if it is a real number and not text
			console.log('is a propper number, will send to chart.')
			plot(plotMqtt, y);	//send it to the plot function
		};
	};

//check if a real number	
	function isNumber(n) {
	  return !isNaN(parseFloat(n)) && isFinite(n);
	};

//function that is called once the document has loaded
	function init() {

		//i find i have to set this to false if i have trouble with timezones.
		Highcharts.setOptions({
			global: { useUTC: false } ,
    time: { timezone: 'America/New_York',  useUTC: false  } 
		});

		// Connect to MQTT broker
		client.connect(options);

	};


//this adds the plots to the chart	
    function plot(point, chartno) {
    	console.log(point);
    	
	        var series = chart.series[0],
	            shift = series.data.length > datapoints; // shift if the series is 
	                                             // longer than max (e.g. 40);
	        // add the point
	        chart.series[chartno].addPoint(point, true, shift);  

	};

//settings for the chart
	$(document).ready(function() {
	    chart = new Highcharts.Chart({
	        chart: {
	            renderTo: 'container',
	            defaultSeriesType: 'spline'
	        },
	        title: {
	            text: 'Plotting Live websockets data from a MQTT topic'
	        },
	        subtitle: {
                                text:  ' | topic : ' + MQTTsubTopic
                        },
	        xAxis: {
	            type: 'datetime',
	            tickPixelInterval: 150,
	            maxZoom: 20 * 1000
	        },
	        yAxis: {
	            minPadding: 0.2,
	            maxPadding: 0.2,
	            title: {
	                text: 'Value',
	                margin: 80
	            }
	        },
	        series: []
	    });        
	});

</script>

<script src="https://cam.pinesec.org/lowcharts/stock/highstock.js"></script>
<script src="https://cam.pinesec.org/lowcharts/modules/exporting.js"></script>

</head>
<body>
<body onload="init();"><!--Start the javascript ball rolling and connect to the mqtt broker-->
<div id="container" style="height: 500px; min-width: 500px"></div><!-- this the placeholder for the chart-->
</BODY></HTML>
)rawliteral";
#endif

#ifdef USE_GRAPHER_BIG
const char mqttws31_js[] PROGMEM = R"rawliteral(
if (typeof Paho === "undefined") {
Paho = {};
}
Paho.MQTT = (function (global) {
// Private variables below, these are only visible inside the function closure
// which is used to define the module.
var version = "@VERSION@";
var buildLevel = "@BUILDLEVEL@";
var MESSAGE_TYPE = {
CONNECT: 1,
CONNACK: 2,
PUBLISH: 3,
PUBACK: 4,
PUBREC: 5,
PUBREL: 6,
PUBCOMP: 7,
SUBSCRIBE: 8,
SUBACK: 9,
UNSUBSCRIBE: 10,
UNSUBACK: 11,
PINGREQ: 12,
PINGRESP: 13,
DISCONNECT: 14
};
var validate = function(obj, keys) {
for (var key in obj) {
if (obj.hasOwnProperty(key)) {
if (keys.hasOwnProperty(key)) {
if (typeof obj[key] !== keys[key])
throw new Error(format(ERROR.INVALID_TYPE, [typeof obj[key], key]));
} else {
var errorStr = "Unknown property, " + key + ". Valid properties are:";
for (var key in keys)
if (keys.hasOwnProperty(key))
errorStr = errorStr+" "+key;
throw new Error(errorStr);
}
}
}
};
var scope = function (f, scope) {
return function () {
return f.apply(scope, arguments);
};
};
var ERROR = {
OK: {code:0, text:"AMQJSC0000I OK."},
CONNECT_TIMEOUT: {code:1, text:"AMQJSC0001E Connect timed out."},
SUBSCRIBE_TIMEOUT: {code:2, text:"AMQJS0002E Subscribe timed out."},
UNSUBSCRIBE_TIMEOUT: {code:3, text:"AMQJS0003E Unsubscribe timed out."},
PING_TIMEOUT: {code:4, text:"AMQJS0004E Ping timed out."},
INTERNAL_ERROR: {code:5, text:"AMQJS0005E Internal error. Error Message: {0}, Stack trace: {1}"},
CONNACK_RETURNCODE: {code:6, text:"AMQJS0006E Bad Connack return code:{0} {1}."},
SOCKET_ERROR: {code:7, text:"AMQJS0007E Socket error:{0}."},
SOCKET_CLOSE: {code:8, text:"AMQJS0008I Socket closed."},
MALFORMED_UTF: {code:9, text:"AMQJS0009E Malformed UTF data:{0} {1} {2}."},
UNSUPPORTED: {code:10, text:"AMQJS0010E {0} is not supported by this browser."},
INVALID_STATE: {code:11, text:"AMQJS0011E Invalid state {0}."},
INVALID_TYPE: {code:12, text:"AMQJS0012E Invalid type {0} for {1}."},
INVALID_ARGUMENT: {code:13, text:"AMQJS0013E Invalid argument {0} for {1}."},
UNSUPPORTED_OPERATION: {code:14, text:"AMQJS0014E Unsupported operation."},
INVALID_STORED_DATA: {code:15, text:"AMQJS0015E Invalid data in local storage key={0} value={1}."},
INVALID_MQTT_MESSAGE_TYPE: {code:16, text:"AMQJS0016E Invalid MQTT message type {0}."},
MALFORMED_UNICODE: {code:17, text:"AMQJS0017E Malformed Unicode string:{0} {1}."},
};
/** CONNACK RC Meaning. */
var CONNACK_RC = {
0:"Connection Accepted",
1:"Connection Refused: unacceptable protocol version",
2:"Connection Refused: identifier rejected",
3:"Connection Refused: server unavailable",
4:"Connection Refused: bad user name or password",
5:"Connection Refused: not authorized"
};
var format = function(error, substitutions) {
var text = error.text;
if (substitutions) {
var field,start;
for (var i=0; i<substitutions.length; i++) {
field = "{"+i+"}";
start = text.indexOf(field);
if(start > 0) {
var part1 = text.substring(0,start);
var part2 = text.substring(start+field.length);
text = part1+substitutions[i]+part2;
}
}
}
return text;
};
//MQTT protocol and version          6    M    Q    I    s    d    p    3
var MqttProtoIdentifierv3 = [0x00,0x06,0x4d,0x51,0x49,0x73,0x64,0x70,0x03];
//MQTT proto/version for 311         4    M    Q    T    T    4
var MqttProtoIdentifierv4 = [0x00,0x04,0x4d,0x51,0x54,0x54,0x04];
var WireMessage = function (type, options) {
this.type = type;
for (var name in options) {
if (options.hasOwnProperty(name)) {
this[name] = options[name];
}
}
};
WireMessage.prototype.encode = function() {
// Compute the first byte of the fixed header
var first = ((this.type & 0x0f) << 4);
/*
* Now calculate the length of the variable header + payload by adding up the lengths
* of all the component parts
*/
var remLength = 0;
var topicStrLength = new Array();
var destinationNameLength = 0;
// if the message contains a messageIdentifier then we need two bytes for that
if (this.messageIdentifier != undefined)
remLength += 2;
switch(this.type) {
// If this a Connect then we need to include 12 bytes for its header
case MESSAGE_TYPE.CONNECT:
switch(this.mqttVersion) {
case 3:
remLength += MqttProtoIdentifierv3.length + 3;
break;
case 4:
remLength += MqttProtoIdentifierv4.length + 3;
break;
}
remLength += UTF8Length(this.clientId) + 2;
if (this.willMessage != undefined) {
remLength += UTF8Length(this.willMessage.destinationName) + 2;
// Will message is always a string, sent as UTF-8 characters with a preceding length.
var willMessagePayloadBytes = this.willMessage.payloadBytes;
if (!(willMessagePayloadBytes instanceof Uint8Array))
willMessagePayloadBytes = new Uint8Array(payloadBytes);
remLength += willMessagePayloadBytes.byteLength +2;
}
if (this.userName != undefined)
remLength += UTF8Length(this.userName) + 2;
if (this.password != undefined)
remLength += UTF8Length(this.password) + 2;
break;
// Subscribe, Unsubscribe can both contain topic strings
case MESSAGE_TYPE.SUBSCRIBE:
first |= 0x02; // Qos = 1;
for ( var i = 0; i < this.topics.length; i++) {
topicStrLength[i] = UTF8Length(this.topics[i]);
remLength += topicStrLength[i] + 2;
}
remLength += this.requestedQos.length; // 1 byte for each topic's Qos
// QoS on Subscribe only
break;
case MESSAGE_TYPE.UNSUBSCRIBE:
first |= 0x02; // Qos = 1;
for ( var i = 0; i < this.topics.length; i++) {
topicStrLength[i] = UTF8Length(this.topics[i]);
remLength += topicStrLength[i] + 2;
}
break;
case MESSAGE_TYPE.PUBREL:
first |= 0x02; // Qos = 1;
break;
case MESSAGE_TYPE.PUBLISH:
if (this.payloadMessage.duplicate) first |= 0x08;
first  = first |= (this.payloadMessage.qos << 1);
if (this.payloadMessage.retained) first |= 0x01;
destinationNameLength = UTF8Length(this.payloadMessage.destinationName);
remLength += destinationNameLength + 2;
var payloadBytes = this.payloadMessage.payloadBytes;
remLength += payloadBytes.byteLength;
if (payloadBytes instanceof ArrayBuffer)
payloadBytes = new Uint8Array(payloadBytes);
else if (!(payloadBytes instanceof Uint8Array))
payloadBytes = new Uint8Array(payloadBytes.buffer);
break;
case MESSAGE_TYPE.DISCONNECT:
break;
default:
;
}
// Now we can allocate a buffer for the message
var mbi = encodeMBI(remLength);  // Convert the length to MQTT MBI format
var pos = mbi.length + 1;        // Offset of start of variable header
var buffer = new ArrayBuffer(remLength + pos);
var byteStream = new Uint8Array(buffer);    // view it as a sequence of bytes
//Write the fixed header into the buffer
byteStream[0] = first;
byteStream.set(mbi,1);
// If this is a PUBLISH then the variable header starts with a topic
if (this.type == MESSAGE_TYPE.PUBLISH)
pos = writeString(this.payloadMessage.destinationName, destinationNameLength, byteStream, pos);
// If this is a CONNECT then the variable header contains the protocol name/version, flags and keepalive time
else if (this.type == MESSAGE_TYPE.CONNECT) {
switch (this.mqttVersion) {
case 3:
byteStream.set(MqttProtoIdentifierv3, pos);
pos += MqttProtoIdentifierv3.length;
break;
case 4:
byteStream.set(MqttProtoIdentifierv4, pos);
pos += MqttProtoIdentifierv4.length;
break;
}
var connectFlags = 0;
if (this.cleanSession)
connectFlags = 0x02;
if (this.willMessage != undefined ) {
connectFlags |= 0x04;
connectFlags |= (this.willMessage.qos<<3);
if (this.willMessage.retained) {
connectFlags |= 0x20;
}
}
if (this.userName != undefined)
connectFlags |= 0x80;
if (this.password != undefined)
connectFlags |= 0x40;
byteStream[pos++] = connectFlags;
pos = writeUint16 (this.keepAliveInterval, byteStream, pos);
}
// Output the messageIdentifier - if there is one
if (this.messageIdentifier != undefined)
pos = writeUint16 (this.messageIdentifier, byteStream, pos);
switch(this.type) {
case MESSAGE_TYPE.CONNECT:
pos = writeString(this.clientId, UTF8Length(this.clientId), byteStream, pos);
if (this.willMessage != undefined) {
pos = writeString(this.willMessage.destinationName, UTF8Length(this.willMessage.destinationName), byteStream, pos);
pos = writeUint16(willMessagePayloadBytes.byteLength, byteStream, pos);
byteStream.set(willMessagePayloadBytes, pos);
pos += willMessagePayloadBytes.byteLength;
}
if (this.userName != undefined)
pos = writeString(this.userName, UTF8Length(this.userName), byteStream, pos);
if (this.password != undefined)
pos = writeString(this.password, UTF8Length(this.password), byteStream, pos);
break;
case MESSAGE_TYPE.PUBLISH:
// PUBLISH has a text or binary payload, if text do not add a 2 byte length field, just the UTF characters.
byteStream.set(payloadBytes, pos);
break;
//    	    case MESSAGE_TYPE.PUBREC:
//    	    case MESSAGE_TYPE.PUBREL:
//    	    case MESSAGE_TYPE.PUBCOMP:
//    	    	break;
case MESSAGE_TYPE.SUBSCRIBE:
// SUBSCRIBE has a list of topic strings and request QoS
for (var i=0; i<this.topics.length; i++) {
pos = writeString(this.topics[i], topicStrLength[i], byteStream, pos);
byteStream[pos++] = this.requestedQos[i];
}
break;
case MESSAGE_TYPE.UNSUBSCRIBE:
// UNSUBSCRIBE has a list of topic strings
for (var i=0; i<this.topics.length; i++)
pos = writeString(this.topics[i], topicStrLength[i], byteStream, pos);
break;
default:
// Do nothing.
}
return buffer;
}
function decodeMessage(input,pos) {
var startingPos = pos;
var first = input[pos];
var type = first >> 4;
var messageInfo = first &= 0x0f;
pos += 1;
// Decode the remaining length (MBI format)
var digit;
var remLength = 0;
var multiplier = 1;
do {
if (pos == input.length) {
return [null,startingPos];
}
digit = input[pos++];
remLength += ((digit & 0x7F) * multiplier);
multiplier *= 128;
} while ((digit & 0x80) != 0);
var endPos = pos+remLength;
if (endPos > input.length) {
return [null,startingPos];
}
var wireMessage = new WireMessage(type);
switch(type) {
case MESSAGE_TYPE.CONNACK:
var connectAcknowledgeFlags = input[pos++];
if (connectAcknowledgeFlags & 0x01)
wireMessage.sessionPresent = true;
wireMessage.returnCode = input[pos++];
break;
case MESSAGE_TYPE.PUBLISH:
var qos = (messageInfo >> 1) & 0x03;
var len = readUint16(input, pos);
pos += 2;
var topicName = parseUTF8(input, pos, len);
pos += len;
// If QoS 1 or 2 there will be a messageIdentifier
if (qos > 0) {
wireMessage.messageIdentifier = readUint16(input, pos);
pos += 2;
}
var message = new Paho.MQTT.Message(input.subarray(pos, endPos));
if ((messageInfo & 0x01) == 0x01)
message.retained = true;
if ((messageInfo & 0x08) == 0x08)
message.duplicate =  true;
message.qos = qos;
message.destinationName = topicName;
wireMessage.payloadMessage = message;
break;
case  MESSAGE_TYPE.PUBACK:
case  MESSAGE_TYPE.PUBREC:
case  MESSAGE_TYPE.PUBREL:
case  MESSAGE_TYPE.PUBCOMP:
case  MESSAGE_TYPE.UNSUBACK:
wireMessage.messageIdentifier = readUint16(input, pos);
break;
case  MESSAGE_TYPE.SUBACK:
wireMessage.messageIdentifier = readUint16(input, pos);
pos += 2;
wireMessage.returnCode = input.subarray(pos, endPos);
break;
default:
;
}
return [wireMessage,endPos];
}
function writeUint16(input, buffer, offset) {
buffer[offset++] = input >> 8;      //MSB
buffer[offset++] = input % 256;     //LSB
return offset;
}
function writeString(input, utf8Length, buffer, offset) {
offset = writeUint16(utf8Length, buffer, offset);
stringToUTF8(input, buffer, offset);
return offset + utf8Length;
}
function readUint16(buffer, offset) {
return 256*buffer[offset] + buffer[offset+1];
}
/**
* Encodes an MQTT Multi-Byte Integer
* @private
*/
function encodeMBI(number) {
var output = new Array(1);
var numBytes = 0;
do {
var digit = number % 128;
number = number >> 7;
if (number > 0) {
digit |= 0x80;
}
output[numBytes++] = digit;
} while ( (number > 0) && (numBytes<4) );
return output;
}
/**
* Takes a String and calculates its length in bytes when encoded in UTF8.
* @private
*/
function UTF8Length(input) {
var output = 0;
for (var i = 0; i<input.length; i++)
{
var charCode = input.charCodeAt(i);
if (charCode > 0x7FF)
{
// Surrogate pair means its a 4 byte character
if (0xD800 <= charCode && charCode <= 0xDBFF)
{
i++;
output++;
}
output +=3;
}
else if (charCode > 0x7F)
output +=2;
else
output++;
}
return output;
}
/**
* Takes a String and writes it into an array as UTF8 encoded bytes.
* @private
*/
function stringToUTF8(input, output, start) {
var pos = start;
for (var i = 0; i<input.length; i++) {
var charCode = input.charCodeAt(i);
// Check for a surrogate pair.
if (0xD800 <= charCode && charCode <= 0xDBFF) {
var lowCharCode = input.charCodeAt(++i);
if (isNaN(lowCharCode)) {
throw new Error(format(ERROR.MALFORMED_UNICODE, [charCode, lowCharCode]));
}
charCode = ((charCode - 0xD800)<<10) + (lowCharCode - 0xDC00) + 0x10000;
}
if (charCode <= 0x7F) {
output[pos++] = charCode;
} else if (charCode <= 0x7FF) {
output[pos++] = charCode>>6  & 0x1F | 0xC0;
output[pos++] = charCode     & 0x3F | 0x80;
} else if (charCode <= 0xFFFF) {
output[pos++] = charCode>>12 & 0x0F | 0xE0;
output[pos++] = charCode>>6  & 0x3F | 0x80;
output[pos++] = charCode     & 0x3F | 0x80;
} else {
output[pos++] = charCode>>18 & 0x07 | 0xF0;
output[pos++] = charCode>>12 & 0x3F | 0x80;
output[pos++] = charCode>>6  & 0x3F | 0x80;
output[pos++] = charCode     & 0x3F | 0x80;
};
}
return output;
}
function parseUTF8(input, offset, length) {
var output = "";
var utf16;
var pos = offset;
while (pos < offset+length)
{
var byte1 = input[pos++];
if (byte1 < 128)
utf16 = byte1;
else
{
var byte2 = input[pos++]-128;
if (byte2 < 0)
throw new Error(format(ERROR.MALFORMED_UTF, [byte1.toString(16), byte2.toString(16),""]));
if (byte1 < 0xE0)             // 2 byte character
utf16 = 64*(byte1-0xC0) + byte2;
else
{
var byte3 = input[pos++]-128;
if (byte3 < 0)
throw new Error(format(ERROR.MALFORMED_UTF, [byte1.toString(16), byte2.toString(16), byte3.toString(16)]));
if (byte1 < 0xF0)        // 3 byte character
utf16 = 4096*(byte1-0xE0) + 64*byte2 + byte3;
else
{
var byte4 = input[pos++]-128;
if (byte4 < 0)
throw new Error(format(ERROR.MALFORMED_UTF, [byte1.toString(16), byte2.toString(16), byte3.toString(16), byte4.toString(16)]));
if (byte1 < 0xF8)        // 4 byte character
utf16 = 262144*(byte1-0xF0) + 4096*byte2 + 64*byte3 + byte4;
else                     // longer encodings are not supported
throw new Error(format(ERROR.MALFORMED_UTF, [byte1.toString(16), byte2.toString(16), byte3.toString(16), byte4.toString(16)]));
}
}
}
if (utf16 > 0xFFFF)   // 4 byte character - express as a surrogate pair
{
utf16 -= 0x10000;
output += String.fromCharCode(0xD800 + (utf16 >> 10)); // lead character
utf16 = 0xDC00 + (utf16 & 0x3FF);  // trail character
}
output += String.fromCharCode(utf16);
}
return output;
}
/**
* Repeat keepalive requests, monitor responses.
* @ignore
*/
var Pinger = function(client, window, keepAliveInterval) {
this._client = client;
this._window = window;
this._keepAliveInterval = keepAliveInterval*1000;
this.isReset = false;
var pingReq = new WireMessage(MESSAGE_TYPE.PINGREQ).encode();
var doTimeout = function (pinger) {
return function () {
return doPing.apply(pinger);
};
};
/** @ignore */
var doPing = function() {
if (!this.isReset) {
this._client._trace("Pinger.doPing", "Timed out");
this._client._disconnected( ERROR.PING_TIMEOUT.code , format(ERROR.PING_TIMEOUT));
} else {
this.isReset = false;
this._client._trace("Pinger.doPing", "send PINGREQ");
this._client.socket.send(pingReq);
this.timeout = this._window.setTimeout(doTimeout(this), this._keepAliveInterval);
}
}
this.reset = function() {
this.isReset = true;
this._window.clearTimeout(this.timeout);
if (this._keepAliveInterval > 0)
this.timeout = setTimeout(doTimeout(this), this._keepAliveInterval);
}
this.cancel = function() {
this._window.clearTimeout(this.timeout);
}
};
/**
* Monitor request completion.
* @ignore
*/
var Timeout = function(client, window, timeoutSeconds, action, args) {
this._window = window;
if (!timeoutSeconds)
timeoutSeconds = 30;
var doTimeout = function (action, client, args) {
return function () {
return action.apply(client, args);
};
};
this.timeout = setTimeout(doTimeout(action, client, args), timeoutSeconds * 1000);
this.cancel = function() {
this._window.clearTimeout(this.timeout);
}
};
var ClientImpl = function (uri, host, port, path, clientId) {
// Check dependencies are satisfied in this browser.
if (!("WebSocket" in global && global["WebSocket"] !== null)) {
throw new Error(format(ERROR.UNSUPPORTED, ["WebSocket"]));
}
if (!("localStorage" in global && global["localStorage"] !== null)) {
throw new Error(format(ERROR.UNSUPPORTED, ["localStorage"]));
}
if (!("ArrayBuffer" in global && global["ArrayBuffer"] !== null)) {
throw new Error(format(ERROR.UNSUPPORTED, ["ArrayBuffer"]));
}
this._trace("Paho.MQTT.Client", uri, host, port, path, clientId);
this.host = host;
this.port = port;
this.path = path;
this.uri = uri;
this.clientId = clientId;
this._localKey=host+":"+port+(path!="/mqtt"?":"+path:"")+":"+clientId+":";
this._msg_queue = [];
this._sentMessages = {};
this._receivedMessages = {};
this._notify_msg_sent = {};
this._message_identifier = 1;
// Used to determine the transmission sequence of stored sent messages.
this._sequence = 0;
// Load the local state, if any, from the saved version, only restore state relevant to this client.
for (var key in localStorage)
if (   key.indexOf("Sent:"+this._localKey) == 0
|| key.indexOf("Received:"+this._localKey) == 0)
this.restore(key);
};
// Messaging Client public instance members.
ClientImpl.prototype.host;
ClientImpl.prototype.port;
ClientImpl.prototype.path;
ClientImpl.prototype.uri;
ClientImpl.prototype.clientId;
// Messaging Client private instance members.
ClientImpl.prototype.socket;
/* true once we have received an acknowledgement to a CONNECT packet. */
ClientImpl.prototype.connected = false;
ClientImpl.prototype.maxMessageIdentifier = 65536;
ClientImpl.prototype.connectOptions;
ClientImpl.prototype.hostIndex;
ClientImpl.prototype.onConnectionLost;
ClientImpl.prototype.onMessageDelivered;
ClientImpl.prototype.onMessageArrived;
ClientImpl.prototype.traceFunction;
ClientImpl.prototype._msg_queue = null;
ClientImpl.prototype._connectTimeout;
/* The sendPinger monitors how long we allow before we send data to prove to the server that we are alive. */
ClientImpl.prototype.sendPinger = null;
/* The receivePinger monitors how long we allow before we require evidence that the server is alive. */
ClientImpl.prototype.receivePinger = null;
ClientImpl.prototype.receiveBuffer = null;
ClientImpl.prototype._traceBuffer = null;
ClientImpl.prototype._MAX_TRACE_ENTRIES = 100;
ClientImpl.prototype.connect = function (connectOptions) {
var connectOptionsMasked = this._traceMask(connectOptions, "password");
this._trace("Client.connect", connectOptionsMasked, this.socket, this.connected);
if (this.connected)
throw new Error(format(ERROR.INVALID_STATE, ["already connected"]));
if (this.socket)
throw new Error(format(ERROR.INVALID_STATE, ["already connected"]));
this.connectOptions = connectOptions;
if (connectOptions.uris) {
this.hostIndex = 0;
this._doConnect(connectOptions.uris[0]);
} else {
this._doConnect(this.uri);
}
};
ClientImpl.prototype.subscribe = function (filter, subscribeOptions) {
this._trace("Client.subscribe", filter, subscribeOptions);
if (!this.connected)
throw new Error(format(ERROR.INVALID_STATE, ["not connected"]));
var wireMessage = new WireMessage(MESSAGE_TYPE.SUBSCRIBE);
wireMessage.topics=[filter];
if (subscribeOptions.qos != undefined)
wireMessage.requestedQos = [subscribeOptions.qos];
else
wireMessage.requestedQos = [0];
if (subscribeOptions.onSuccess) {
wireMessage.onSuccess = function(grantedQos) {subscribeOptions.onSuccess({invocationContext:subscribeOptions.invocationContext,grantedQos:grantedQos});};
}
if (subscribeOptions.onFailure) {
wireMessage.onFailure = function(errorCode) {subscribeOptions.onFailure({invocationContext:subscribeOptions.invocationContext,errorCode:errorCode});};
}
if (subscribeOptions.timeout) {
wireMessage.timeOut = new Timeout(this, window, subscribeOptions.timeout, subscribeOptions.onFailure
, [{invocationContext:subscribeOptions.invocationContext,
errorCode:ERROR.SUBSCRIBE_TIMEOUT.code,
errorMessage:format(ERROR.SUBSCRIBE_TIMEOUT)}]);
}
// All subscriptions return a SUBACK.
this._requires_ack(wireMessage);
this._schedule_message(wireMessage);
};
/** @ignore */
ClientImpl.prototype.unsubscribe = function(filter, unsubscribeOptions) {
this._trace("Client.unsubscribe", filter, unsubscribeOptions);
if (!this.connected)
throw new Error(format(ERROR.INVALID_STATE, ["not connected"]));
var wireMessage = new WireMessage(MESSAGE_TYPE.UNSUBSCRIBE);
wireMessage.topics = [filter];
if (unsubscribeOptions.onSuccess) {
wireMessage.callback = function() {unsubscribeOptions.onSuccess({invocationContext:unsubscribeOptions.invocationContext});};
}
if (unsubscribeOptions.timeout) {
wireMessage.timeOut = new Timeout(this, window, unsubscribeOptions.timeout, unsubscribeOptions.onFailure
, [{invocationContext:unsubscribeOptions.invocationContext,
errorCode:ERROR.UNSUBSCRIBE_TIMEOUT.code,
errorMessage:format(ERROR.UNSUBSCRIBE_TIMEOUT)}]);
}
// All unsubscribes return a SUBACK.
this._requires_ack(wireMessage);
this._schedule_message(wireMessage);
};
ClientImpl.prototype.send = function (message) {
this._trace("Client.send", message);
if (!this.connected)
throw new Error(format(ERROR.INVALID_STATE, ["not connected"]));
wireMessage = new WireMessage(MESSAGE_TYPE.PUBLISH);
wireMessage.payloadMessage = message;
if (message.qos > 0)
this._requires_ack(wireMessage);
else if (this.onMessageDelivered)
this._notify_msg_sent[wireMessage] = this.onMessageDelivered(wireMessage.payloadMessage);
this._schedule_message(wireMessage);
};
ClientImpl.prototype.disconnect = function () {
this._trace("Client.disconnect");
if (!this.socket)
throw new Error(format(ERROR.INVALID_STATE, ["not connecting or connected"]));
wireMessage = new WireMessage(MESSAGE_TYPE.DISCONNECT);
this._notify_msg_sent[wireMessage] = scope(this._disconnected, this);
this._schedule_message(wireMessage);
};
ClientImpl.prototype.getTraceLog = function () {
if ( this._traceBuffer !== null ) {
this._trace("Client.getTraceLog", new Date());
this._trace("Client.getTraceLog in flight messages", this._sentMessages.length);
for (var key in this._sentMessages)
this._trace("_sentMessages ",key, this._sentMessages[key]);
for (var key in this._receivedMessages)
this._trace("_receivedMessages ",key, this._receivedMessages[key]);
return this._traceBuffer;
}
};
ClientImpl.prototype.startTrace = function () {
if ( this._traceBuffer === null ) {
this._traceBuffer = [];
}
this._trace("Client.startTrace", new Date(), version);
};
ClientImpl.prototype.stopTrace = function () {
delete this._traceBuffer;
};
ClientImpl.prototype._doConnect = function (wsurl) {
// When the socket is open, this client will send the CONNECT WireMessage using the saved parameters.
if (this.connectOptions.useSSL) {
var uriParts = wsurl.split(":");
uriParts[0] = "wss";
wsurl = uriParts.join(":");
}
this.connected = false;
if (this.connectOptions.mqttVersion < 4) {
this.socket = new WebSocket(wsurl, ["mqttv3.1"]);
} else {
this.socket = new WebSocket(wsurl, ["mqtt"]);
}
this.socket.binaryType = 'arraybuffer';
this.socket.onopen = scope(this._on_socket_open, this);
this.socket.onmessage = scope(this._on_socket_message, this);
this.socket.onerror = scope(this._on_socket_error, this);
this.socket.onclose = scope(this._on_socket_close, this);
this.sendPinger = new Pinger(this, window, this.connectOptions.keepAliveInterval);
this.receivePinger = new Pinger(this, window, this.connectOptions.keepAliveInterval);
this._connectTimeout = new Timeout(this, window, this.connectOptions.timeout, this._disconnected,  [ERROR.CONNECT_TIMEOUT.code, format(ERROR.CONNECT_TIMEOUT)]);
};
ClientImpl.prototype._schedule_message = function (message) {
this._msg_queue.push(message);
// Process outstanding messages in the queue if we have an  open socket, and have received CONNACK.
if (this.connected) {
this._process_queue();
}
};
ClientImpl.prototype.store = function(prefix, wireMessage) {
var storedMessage = {type:wireMessage.type, messageIdentifier:wireMessage.messageIdentifier, version:1};
switch(wireMessage.type) {
case MESSAGE_TYPE.PUBLISH:
if(wireMessage.pubRecReceived)
storedMessage.pubRecReceived = true;
// Convert the payload to a hex string.
storedMessage.payloadMessage = {};
var hex = "";
var messageBytes = wireMessage.payloadMessage.payloadBytes;
for (var i=0; i<messageBytes.length; i++) {
if (messageBytes[i] <= 0xF)
hex = hex+"0"+messageBytes[i].toString(16);
else
hex = hex+messageBytes[i].toString(16);
}
storedMessage.payloadMessage.payloadHex = hex;
storedMessage.payloadMessage.qos = wireMessage.payloadMessage.qos;
storedMessage.payloadMessage.destinationName = wireMessage.payloadMessage.destinationName;
if (wireMessage.payloadMessage.duplicate)
storedMessage.payloadMessage.duplicate = true;
if (wireMessage.payloadMessage.retained)
storedMessage.payloadMessage.retained = true;
// Add a sequence number to sent messages.
if ( prefix.indexOf("Sent:") == 0 ) {
if ( wireMessage.sequence === undefined )
wireMessage.sequence = ++this._sequence;
storedMessage.sequence = wireMessage.sequence;
}
break;
default:
throw Error(format(ERROR.INVALID_STORED_DATA, [key, storedMessage]));
}
localStorage.setItem(prefix+this._localKey+wireMessage.messageIdentifier, JSON.stringify(storedMessage));
};
ClientImpl.prototype.restore = function(key) {
var value = localStorage.getItem(key);
var storedMessage = JSON.parse(value);
var wireMessage = new WireMessage(storedMessage.type, storedMessage);
switch(storedMessage.type) {
case MESSAGE_TYPE.PUBLISH:
// Replace the payload message with a Message object.
var hex = storedMessage.payloadMessage.payloadHex;
var buffer = new ArrayBuffer((hex.length)/2);
var byteStream = new Uint8Array(buffer);
var i = 0;
while (hex.length >= 2) {
var x = parseInt(hex.substring(0, 2), 16);
hex = hex.substring(2, hex.length);
byteStream[i++] = x;
}
var payloadMessage = new Paho.MQTT.Message(byteStream);
payloadMessage.qos = storedMessage.payloadMessage.qos;
payloadMessage.destinationName = storedMessage.payloadMessage.destinationName;
if (storedMessage.payloadMessage.duplicate)
payloadMessage.duplicate = true;
if (storedMessage.payloadMessage.retained)
payloadMessage.retained = true;
wireMessage.payloadMessage = payloadMessage;
break;
default:
throw Error(format(ERROR.INVALID_STORED_DATA, [key, value]));
}
if (key.indexOf("Sent:"+this._localKey) == 0) {
wireMessage.payloadMessage.duplicate = true;
this._sentMessages[wireMessage.messageIdentifier] = wireMessage;
} else if (key.indexOf("Received:"+this._localKey) == 0) {
this._receivedMessages[wireMessage.messageIdentifier] = wireMessage;
}
};
ClientImpl.prototype._process_queue = function () {
var message = null;
// Process messages in order they were added
var fifo = this._msg_queue.reverse();
// Send all queued messages down socket connection
while ((message = fifo.pop())) {
this._socket_send(message);
// Notify listeners that message was successfully sent
if (this._notify_msg_sent[message]) {
this._notify_msg_sent[message]();
delete this._notify_msg_sent[message];
}
}
};
ClientImpl.prototype._requires_ack = function (wireMessage) {
var messageCount = Object.keys(this._sentMessages).length;
if (messageCount > this.maxMessageIdentifier)
throw Error ("Too many messages:"+messageCount);
while(this._sentMessages[this._message_identifier] !== undefined) {
this._message_identifier++;
}
wireMessage.messageIdentifier = this._message_identifier;
this._sentMessages[wireMessage.messageIdentifier] = wireMessage;
if (wireMessage.type === MESSAGE_TYPE.PUBLISH) {
this.store("Sent:", wireMessage);
}
if (this._message_identifier === this.maxMessageIdentifier) {
this._message_identifier = 1;
}
};
ClientImpl.prototype._on_socket_open = function () {
// Create the CONNECT message object.
var wireMessage = new WireMessage(MESSAGE_TYPE.CONNECT, this.connectOptions);
wireMessage.clientId = this.clientId;
this._socket_send(wireMessage);
};
ClientImpl.prototype._on_socket_message = function (event) {
this._trace("Client._on_socket_message", event.data);
// Reset the receive ping timer, we now have evidence the server is alive.
this.receivePinger.reset();
var messages = this._deframeMessages(event.data);
for (var i = 0; i < messages.length; i+=1) {
this._handleMessage(messages[i]);
}
}
ClientImpl.prototype._deframeMessages = function(data) {
var byteArray = new Uint8Array(data);
if (this.receiveBuffer) {
var newData = new Uint8Array(this.receiveBuffer.length+byteArray.length);
newData.set(this.receiveBuffer);
newData.set(byteArray,this.receiveBuffer.length);
byteArray = newData;
delete this.receiveBuffer;
}
try {
var offset = 0;
var messages = [];
while(offset < byteArray.length) {
var result = decodeMessage(byteArray,offset);
var wireMessage = result[0];
offset = result[1];
if (wireMessage !== null) {
messages.push(wireMessage);
} else {
break;
}
}
if (offset < byteArray.length) {
this.receiveBuffer = byteArray.subarray(offset);
}
} catch (error) {
this._disconnected(ERROR.INTERNAL_ERROR.code , format(ERROR.INTERNAL_ERROR, [error.message,error.stack.toString()]));
return;
}
return messages;
}
ClientImpl.prototype._handleMessage = function(wireMessage) {
this._trace("Client._handleMessage", wireMessage);
try {
switch(wireMessage.type) {
case MESSAGE_TYPE.CONNACK:
this._connectTimeout.cancel();
// If we have started using clean session then clear up the local state.
if (this.connectOptions.cleanSession) {
for (var key in this._sentMessages) {
var sentMessage = this._sentMessages[key];
localStorage.removeItem("Sent:"+this._localKey+sentMessage.messageIdentifier);
}
this._sentMessages = {};
for (var key in this._receivedMessages) {
var receivedMessage = this._receivedMessages[key];
localStorage.removeItem("Received:"+this._localKey+receivedMessage.messageIdentifier);
}
this._receivedMessages = {};
}
// Client connected and ready for business.
if (wireMessage.returnCode === 0) {
this.connected = true;
// Jump to the end of the list of uris and stop looking for a good host.
if (this.connectOptions.uris)
this.hostIndex = this.connectOptions.uris.length;
} else {
this._disconnected(ERROR.CONNACK_RETURNCODE.code , format(ERROR.CONNACK_RETURNCODE, [wireMessage.returnCode, CONNACK_RC[wireMessage.returnCode]]));
break;
}
// Resend messages.
var sequencedMessages = new Array();
for (var msgId in this._sentMessages) {
if (this._sentMessages.hasOwnProperty(msgId))
sequencedMessages.push(this._sentMessages[msgId]);
}
// Sort sentMessages into the original sent order.
var sequencedMessages = sequencedMessages.sort(function(a,b) {return a.sequence - b.sequence;} );
for (var i=0, len=sequencedMessages.length; i<len; i++) {
var sentMessage = sequencedMessages[i];
if (sentMessage.type == MESSAGE_TYPE.PUBLISH && sentMessage.pubRecReceived) {
var pubRelMessage = new WireMessage(MESSAGE_TYPE.PUBREL, {messageIdentifier:sentMessage.messageIdentifier});
this._schedule_message(pubRelMessage);
} else {
this._schedule_message(sentMessage);
};
}
// Execute the connectOptions.onSuccess callback if there is one.
if (this.connectOptions.onSuccess) {
this.connectOptions.onSuccess({invocationContext:this.connectOptions.invocationContext});
}
// Process all queued messages now that the connection is established.
this._process_queue();
break;
case MESSAGE_TYPE.PUBLISH:
this._receivePublish(wireMessage);
break;
case MESSAGE_TYPE.PUBACK:
var sentMessage = this._sentMessages[wireMessage.messageIdentifier];
// If this is a re flow of a PUBACK after we have restarted receivedMessage will not exist.
if (sentMessage) {
delete this._sentMessages[wireMessage.messageIdentifier];
localStorage.removeItem("Sent:"+this._localKey+wireMessage.messageIdentifier);
if (this.onMessageDelivered)
this.onMessageDelivered(sentMessage.payloadMessage);
}
break;
case MESSAGE_TYPE.PUBREC:
var sentMessage = this._sentMessages[wireMessage.messageIdentifier];
// If this is a re flow of a PUBREC after we have restarted receivedMessage will not exist.
if (sentMessage) {
sentMessage.pubRecReceived = true;
var pubRelMessage = new WireMessage(MESSAGE_TYPE.PUBREL, {messageIdentifier:wireMessage.messageIdentifier});
this.store("Sent:", sentMessage);
this._schedule_message(pubRelMessage);
}
break;
case MESSAGE_TYPE.PUBREL:
var receivedMessage = this._receivedMessages[wireMessage.messageIdentifier];
localStorage.removeItem("Received:"+this._localKey+wireMessage.messageIdentifier);
// If this is a re flow of a PUBREL after we have restarted receivedMessage will not exist.
if (receivedMessage) {
this._receiveMessage(receivedMessage);
delete this._receivedMessages[wireMessage.messageIdentifier];
}
// Always flow PubComp, we may have previously flowed PubComp but the server lost it and restarted.
var pubCompMessage = new WireMessage(MESSAGE_TYPE.PUBCOMP, {messageIdentifier:wireMessage.messageIdentifier});
this._schedule_message(pubCompMessage);
break;
case MESSAGE_TYPE.PUBCOMP:
var sentMessage = this._sentMessages[wireMessage.messageIdentifier];
delete this._sentMessages[wireMessage.messageIdentifier];
localStorage.removeItem("Sent:"+this._localKey+wireMessage.messageIdentifier);
if (this.onMessageDelivered)
this.onMessageDelivered(sentMessage.payloadMessage);
break;
case MESSAGE_TYPE.SUBACK:
var sentMessage = this._sentMessages[wireMessage.messageIdentifier];
if (sentMessage) {
if(sentMessage.timeOut)
sentMessage.timeOut.cancel();
// This will need to be fixed when we add multiple topic support
if (wireMessage.returnCode[0] === 0x80) {
if (sentMessage.onFailure) {
sentMessage.onFailure(wireMessage.returnCode);
}
} else if (sentMessage.onSuccess) {
sentMessage.onSuccess(wireMessage.returnCode);
}
delete this._sentMessages[wireMessage.messageIdentifier];
}
break;
case MESSAGE_TYPE.UNSUBACK:
var sentMessage = this._sentMessages[wireMessage.messageIdentifier];
if (sentMessage) {
if (sentMessage.timeOut)
sentMessage.timeOut.cancel();
if (sentMessage.callback) {
sentMessage.callback();
}
delete this._sentMessages[wireMessage.messageIdentifier];
}
break;
case MESSAGE_TYPE.PINGRESP:
/* The sendPinger or receivePinger may have sent a ping, the receivePinger has already been reset. */
this.sendPinger.reset();
break;
case MESSAGE_TYPE.DISCONNECT:
// Clients do not expect to receive disconnect packets.
this._disconnected(ERROR.INVALID_MQTT_MESSAGE_TYPE.code , format(ERROR.INVALID_MQTT_MESSAGE_TYPE, [wireMessage.type]));
break;
default:
this._disconnected(ERROR.INVALID_MQTT_MESSAGE_TYPE.code , format(ERROR.INVALID_MQTT_MESSAGE_TYPE, [wireMessage.type]));
};
} catch (error) {
this._disconnected(ERROR.INTERNAL_ERROR.code , format(ERROR.INTERNAL_ERROR, [error.message,error.stack.toString()]));
return;
}
};
/** @ignore */
ClientImpl.prototype._on_socket_error = function (error) {
this._disconnected(ERROR.SOCKET_ERROR.code , format(ERROR.SOCKET_ERROR, [error.data]));
};
/** @ignore */
ClientImpl.prototype._on_socket_close = function () {
this._disconnected(ERROR.SOCKET_CLOSE.code , format(ERROR.SOCKET_CLOSE));
};
/** @ignore */
ClientImpl.prototype._socket_send = function (wireMessage) {
if (wireMessage.type == 1) {
var wireMessageMasked = this._traceMask(wireMessage, "password");
this._trace("Client._socket_send", wireMessageMasked);
}
else this._trace("Client._socket_send", wireMessage);
this.socket.send(wireMessage.encode());
/* We have proved to the server we are alive. */
this.sendPinger.reset();
};
/** @ignore */
ClientImpl.prototype._receivePublish = function (wireMessage) {
switch(wireMessage.payloadMessage.qos) {
case "undefined":
case 0:
this._receiveMessage(wireMessage);
break;
case 1:
var pubAckMessage = new WireMessage(MESSAGE_TYPE.PUBACK, {messageIdentifier:wireMessage.messageIdentifier});
this._schedule_message(pubAckMessage);
this._receiveMessage(wireMessage);
break;
case 2:
this._receivedMessages[wireMessage.messageIdentifier] = wireMessage;
this.store("Received:", wireMessage);
var pubRecMessage = new WireMessage(MESSAGE_TYPE.PUBREC, {messageIdentifier:wireMessage.messageIdentifier});
this._schedule_message(pubRecMessage);
break;
default:
throw Error("Invaild qos="+wireMmessage.payloadMessage.qos);
};
};
/** @ignore */
ClientImpl.prototype._receiveMessage = function (wireMessage) {
if (this.onMessageArrived) {
this.onMessageArrived(wireMessage.payloadMessage);
}
};
ClientImpl.prototype._disconnected = function (errorCode, errorText) {
this._trace("Client._disconnected", errorCode, errorText);
this.sendPinger.cancel();
this.receivePinger.cancel();
if (this._connectTimeout)
this._connectTimeout.cancel();
// Clear message buffers.
this._msg_queue = [];
this._notify_msg_sent = {};
if (this.socket) {
// Cancel all socket callbacks so that they cannot be driven again by this socket.
this.socket.onopen = null;
this.socket.onmessage = null;
this.socket.onerror = null;
this.socket.onclose = null;
if (this.socket.readyState === 1)
this.socket.close();
delete this.socket;
}
if (this.connectOptions.uris && this.hostIndex < this.connectOptions.uris.length-1) {
// Try the next host.
this.hostIndex++;
this._doConnect(this.connectOptions.uris[this.hostIndex]);
} else {
if (errorCode === undefined) {
errorCode = ERROR.OK.code;
errorText = format(ERROR.OK);
}
// Run any application callbacks last as they may attempt to reconnect and hence create a new socket.
if (this.connected) {
this.connected = false;
// Execute the connectionLostCallback if there is one, and we were connected.
if (this.onConnectionLost)
this.onConnectionLost({errorCode:errorCode, errorMessage:errorText});
} else {
// Otherwise we never had a connection, so indicate that the connect has failed.
if (this.connectOptions.mqttVersion === 4 && this.connectOptions.mqttVersionExplicit === false) {
this._trace("Failed to connect V4, dropping back to V3")
this.connectOptions.mqttVersion = 3;
if (this.connectOptions.uris) {
this.hostIndex = 0;
this._doConnect(this.connectOptions.uris[0]);
} else {
this._doConnect(this.uri);
}
} else if(this.connectOptions.onFailure) {
this.connectOptions.onFailure({invocationContext:this.connectOptions.invocationContext, errorCode:errorCode, errorMessage:errorText});
}
}
}
};
/** @ignore */
ClientImpl.prototype._trace = function () {
// Pass trace message back to client's callback function
if (this.traceFunction) {
for (var i in arguments)
{
if (typeof arguments[i] !== "undefined")
arguments[i] = JSON.stringify(arguments[i]);
}
var record = Array.prototype.slice.call(arguments).join("");
this.traceFunction ({severity: "Debug", message: record	});
}
//buffer style trace
if ( this._traceBuffer !== null ) {
for (var i = 0, max = arguments.length; i < max; i++) {
if ( this._traceBuffer.length == this._MAX_TRACE_ENTRIES ) {
this._traceBuffer.shift();
}
if (i === 0) this._traceBuffer.push(arguments[i]);
else if (typeof arguments[i] === "undefined" ) this._traceBuffer.push(arguments[i]);
else this._traceBuffer.push("  "+JSON.stringify(arguments[i]));
};
};
};
/** @ignore */
ClientImpl.prototype._traceMask = function (traceObject, masked) {
var traceObjectMasked = {};
for (var attr in traceObject) {
if (traceObject.hasOwnProperty(attr)) {
if (attr == masked)
traceObjectMasked[attr] = "******";
else
traceObjectMasked[attr] = traceObject[attr];
}
}
return traceObjectMasked;
};
var Client = function (host, port, path, clientId) {
var uri;
if (typeof host !== "string")
throw new Error(format(ERROR.INVALID_TYPE, [typeof host, "host"]));
if (arguments.length == 2) {
// host: must be full ws:// uri
// port: clientId
clientId = port;
uri = host;
var match = uri.match(/^(wss?):\/\/((\[(.+)\])|([^\/]+?))(:(\d+))?(\/.*)$/);
if (match) {
host = match[4]||match[2];
port = parseInt(match[7]);
path = match[8];
} else {
throw new Error(format(ERROR.INVALID_ARGUMENT,[host,"host"]));
}
} else {
if (arguments.length == 3) {
clientId = path;
path = "/mqtt";
}
if (typeof port !== "number" || port < 0)
throw new Error(format(ERROR.INVALID_TYPE, [typeof port, "port"]));
if (typeof path !== "string")
throw new Error(format(ERROR.INVALID_TYPE, [typeof path, "path"]));
var ipv6AddSBracket = (host.indexOf(":") != -1 && host.slice(0,1) != "[" && host.slice(-1) != "]");
uri = "ws://"+(ipv6AddSBracket?"["+host+"]":host)+":"+port+path;
}
var clientIdLength = 0;
for (var i = 0; i<clientId.length; i++) {
var charCode = clientId.charCodeAt(i);
if (0xD800 <= charCode && charCode <= 0xDBFF)  {
i++; // Surrogate pair.
}
clientIdLength++;
}
if (typeof clientId !== "string" || clientIdLength > 65535)
throw new Error(format(ERROR.INVALID_ARGUMENT, [clientId, "clientId"]));
var client = new ClientImpl(uri, host, port, path, clientId);
this._getHost =  function() { return host; };
this._setHost = function() { throw new Error(format(ERROR.UNSUPPORTED_OPERATION)); };
this._getPort = function() { return port; };
this._setPort = function() { throw new Error(format(ERROR.UNSUPPORTED_OPERATION)); };
this._getPath = function() { return path; };
this._setPath = function() { throw new Error(format(ERROR.UNSUPPORTED_OPERATION)); };
this._getURI = function() { return uri; };
this._setURI = function() { throw new Error(format(ERROR.UNSUPPORTED_OPERATION)); };
this._getClientId = function() { return client.clientId; };
this._setClientId = function() { throw new Error(format(ERROR.UNSUPPORTED_OPERATION)); };
this._getOnConnectionLost = function() { return client.onConnectionLost; };
this._setOnConnectionLost = function(newOnConnectionLost) {
if (typeof newOnConnectionLost === "function")
client.onConnectionLost = newOnConnectionLost;
else
throw new Error(format(ERROR.INVALID_TYPE, [typeof newOnConnectionLost, "onConnectionLost"]));
};
this._getOnMessageDelivered = function() { return client.onMessageDelivered; };
this._setOnMessageDelivered = function(newOnMessageDelivered) {
if (typeof newOnMessageDelivered === "function")
client.onMessageDelivered = newOnMessageDelivered;
else
throw new Error(format(ERROR.INVALID_TYPE, [typeof newOnMessageDelivered, "onMessageDelivered"]));
};
this._getOnMessageArrived = function() { return client.onMessageArrived; };
this._setOnMessageArrived = function(newOnMessageArrived) {
if (typeof newOnMessageArrived === "function")
client.onMessageArrived = newOnMessageArrived;
else
throw new Error(format(ERROR.INVALID_TYPE, [typeof newOnMessageArrived, "onMessageArrived"]));
};
this._getTrace = function() { return client.traceFunction; };
this._setTrace = function(trace) {
if(typeof trace === "function"){
client.traceFunction = trace;
}else{
throw new Error(format(ERROR.INVALID_TYPE, [typeof trace, "onTrace"]));
}
};
this.connect = function (connectOptions) {
connectOptions = connectOptions || {} ;
validate(connectOptions,  {timeout:"number",
userName:"string",
password:"string",
willMessage:"object",
keepAliveInterval:"number",
cleanSession:"boolean",
useSSL:"boolean",
invocationContext:"object",
onSuccess:"function",
onFailure:"function",
hosts:"object",
ports:"object",
mqttVersion:"number"});
// If no keep alive interval is set, assume 60 seconds.
if (connectOptions.keepAliveInterval === undefined)
connectOptions.keepAliveInterval = 60;
if (connectOptions.mqttVersion > 4 || connectOptions.mqttVersion < 3) {
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.mqttVersion, "connectOptions.mqttVersion"]));
}
if (connectOptions.mqttVersion === undefined) {
connectOptions.mqttVersionExplicit = false;
connectOptions.mqttVersion = 4;
} else {
connectOptions.mqttVersionExplicit = true;
}
//Check that if password is set, so is username
if (connectOptions.password === undefined && connectOptions.userName !== undefined)
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.password, "connectOptions.password"]))
if (connectOptions.willMessage) {
if (!(connectOptions.willMessage instanceof Message))
throw new Error(format(ERROR.INVALID_TYPE, [connectOptions.willMessage, "connectOptions.willMessage"]));
// The will message must have a payload that can be represented as a string.
// Cause the willMessage to throw an exception if this is not the case.
connectOptions.willMessage.stringPayload;
if (typeof connectOptions.willMessage.destinationName === "undefined")
throw new Error(format(ERROR.INVALID_TYPE, [typeof connectOptions.willMessage.destinationName, "connectOptions.willMessage.destinationName"]));
}
if (typeof connectOptions.cleanSession === "undefined")
connectOptions.cleanSession = true;
if (connectOptions.hosts) {
if (!(connectOptions.hosts instanceof Array) )
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.hosts, "connectOptions.hosts"]));
if (connectOptions.hosts.length <1 )
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.hosts, "connectOptions.hosts"]));
var usingURIs = false;
for (var i = 0; i<connectOptions.hosts.length; i++) {
if (typeof connectOptions.hosts[i] !== "string")
throw new Error(format(ERROR.INVALID_TYPE, [typeof connectOptions.hosts[i], "connectOptions.hosts["+i+"]"]));
if (/^(wss?):\/\/((\[(.+)\])|([^\/]+?))(:(\d+))?(\/.*)$/.test(connectOptions.hosts[i])) {
if (i == 0) {
usingURIs = true;
} else if (!usingURIs) {
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.hosts[i], "connectOptions.hosts["+i+"]"]));
}
} else if (usingURIs) {
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.hosts[i], "connectOptions.hosts["+i+"]"]));
}
}
if (!usingURIs) {
if (!connectOptions.ports)
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.ports, "connectOptions.ports"]));
if (!(connectOptions.ports instanceof Array) )
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.ports, "connectOptions.ports"]));
if (connectOptions.hosts.length != connectOptions.ports.length)
throw new Error(format(ERROR.INVALID_ARGUMENT, [connectOptions.ports, "connectOptions.ports"]));
connectOptions.uris = [];
for (var i = 0; i<connectOptions.hosts.length; i++) {
if (typeof connectOptions.ports[i] !== "number" || connectOptions.ports[i] < 0)
throw new Error(format(ERROR.INVALID_TYPE, [typeof connectOptions.ports[i], "connectOptions.ports["+i+"]"]));
var host = connectOptions.hosts[i];
var port = connectOptions.ports[i];
var ipv6 = (host.indexOf(":") != -1);
uri = "ws://"+(ipv6?"["+host+"]":host)+":"+port+path;
connectOptions.uris.push(uri);
}
} else {
connectOptions.uris = connectOptions.hosts;
}
}
client.connect(connectOptions);
};
this.subscribe = function (filter, subscribeOptions) {
if (typeof filter !== "string")
throw new Error("Invalid argument:"+filter);
subscribeOptions = subscribeOptions || {} ;
validate(subscribeOptions,  {qos:"number",
invocationContext:"object",
onSuccess:"function",
onFailure:"function",
timeout:"number"
});
if (subscribeOptions.timeout && !subscribeOptions.onFailure)
throw new Error("subscribeOptions.timeout specified with no onFailure callback.");
if (typeof subscribeOptions.qos !== "undefined"
&& !(subscribeOptions.qos === 0 || subscribeOptions.qos === 1 || subscribeOptions.qos === 2 ))
throw new Error(format(ERROR.INVALID_ARGUMENT, [subscribeOptions.qos, "subscribeOptions.qos"]));
client.subscribe(filter, subscribeOptions);
};
this.unsubscribe = function (filter, unsubscribeOptions) {
if (typeof filter !== "string")
throw new Error("Invalid argument:"+filter);
unsubscribeOptions = unsubscribeOptions || {} ;
validate(unsubscribeOptions,  {invocationContext:"object",
onSuccess:"function",
onFailure:"function",
timeout:"number"
});
if (unsubscribeOptions.timeout && !unsubscribeOptions.onFailure)
throw new Error("unsubscribeOptions.timeout specified with no onFailure callback.");
client.unsubscribe(filter, unsubscribeOptions);
};
 
this.send = function (topic,payload,qos,retained) {
var message ;
if(arguments.length == 0){
throw new Error("Invalid argument."+"length");
}else if(arguments.length == 1) {
if (!(topic instanceof Message) && (typeof topic !== "string"))
throw new Error("Invalid argument:"+ typeof topic);
message = topic;
if (typeof message.destinationName === "undefined")
throw new Error(format(ERROR.INVALID_ARGUMENT,[message.destinationName,"Message.destinationName"]));
client.send(message);
}else {
//parameter checking in Message object
message = new Message(payload);
message.destinationName = topic;
if(arguments.length >= 3)
message.qos = qos;
if(arguments.length >= 4)
message.retained = retained;
client.send(message);
}
};
 
this.disconnect = function () {
client.disconnect();
};
 
this.getTraceLog = function () {
return client.getTraceLog();
}
 
this.startTrace = function () {
client.startTrace();
};
 
this.stopTrace = function () {
client.stopTrace();
};
this.isConnected = function() {
return client.connected;
};
};
Client.prototype = {
get host() { return this._getHost(); },
set host(newHost) { this._setHost(newHost); },
get port() { return this._getPort(); },
set port(newPort) { this._setPort(newPort); },
get path() { return this._getPath(); },
set path(newPath) { this._setPath(newPath); },
get clientId() { return this._getClientId(); },
set clientId(newClientId) { this._setClientId(newClientId); },
get onConnectionLost() { return this._getOnConnectionLost(); },
set onConnectionLost(newOnConnectionLost) { this._setOnConnectionLost(newOnConnectionLost); },
get onMessageDelivered() { return this._getOnMessageDelivered(); },
set onMessageDelivered(newOnMessageDelivered) { this._setOnMessageDelivered(newOnMessageDelivered); },
get onMessageArrived() { return this._getOnMessageArrived(); },
set onMessageArrived(newOnMessageArrived) { this._setOnMessageArrived(newOnMessageArrived); },
get trace() { return this._getTrace(); },
set trace(newTraceFunction) { this._setTrace(newTraceFunction); }
};
var Message = function (newPayload) {
var payload;
if (   typeof newPayload === "string"
|| newPayload instanceof ArrayBuffer
|| newPayload instanceof Int8Array
|| newPayload instanceof Uint8Array
|| newPayload instanceof Int16Array
|| newPayload instanceof Uint16Array
|| newPayload instanceof Int32Array
|| newPayload instanceof Uint32Array
|| newPayload instanceof Float32Array
|| newPayload instanceof Float64Array
) {
payload = newPayload;
} else {
throw (format(ERROR.INVALID_ARGUMENT, [newPayload, "newPayload"]));
}
this._getPayloadString = function () {
if (typeof payload === "string")
return payload;
else
return parseUTF8(payload, 0, payload.length);
};
this._getPayloadBytes = function() {
if (typeof payload === "string") {
var buffer = new ArrayBuffer(UTF8Length(payload));
var byteStream = new Uint8Array(buffer);
stringToUTF8(payload, byteStream, 0);
return byteStream;
} else {
return payload;
};
};
var destinationName = undefined;
this._getDestinationName = function() { return destinationName; };
this._setDestinationName = function(newDestinationName) {
if (typeof newDestinationName === "string")
destinationName = newDestinationName;
else
throw new Error(format(ERROR.INVALID_ARGUMENT, [newDestinationName, "newDestinationName"]));
};
var qos = 0;
this._getQos = function() { return qos; };
this._setQos = function(newQos) {
if (newQos === 0 || newQos === 1 || newQos === 2 )
qos = newQos;
else
throw new Error("Invalid argument:"+newQos);
};
var retained = false;
this._getRetained = function() { return retained; };
this._setRetained = function(newRetained) {
if (typeof newRetained === "boolean")
retained = newRetained;
else
throw new Error(format(ERROR.INVALID_ARGUMENT, [newRetained, "newRetained"]));
};
var duplicate = false;
this._getDuplicate = function() { return duplicate; };
this._setDuplicate = function(newDuplicate) { duplicate = newDuplicate; };
};
Message.prototype = {
get payloadString() { return this._getPayloadString(); },
get payloadBytes() { return this._getPayloadBytes(); },
get destinationName() { return this._getDestinationName(); },
set destinationName(newDestinationName) { this._setDestinationName(newDestinationName); },
get qos() { return this._getQos(); },
set qos(newQos) { this._setQos(newQos); },
get retained() { return this._getRetained(); },
set retained(newRetained) { this._setRetained(newRetained); },
get duplicate() { return this._getDuplicate(); },
set duplicate(newDuplicate) { this._setDuplicate(newDuplicate); }
};
// Module contents.
return {
Client: Client,
Message: Message
};
})(window);
)rawliteral";
#endif

#ifdef USE_GRAPHER
const char mqttws31_js[] PROGMEM = R"rawliteral(
"undefined"==typeof Paho&&(Paho={}),Paho.MQTT=function(e){var t=1,s=2,n=3,i=4,o=5,r=6,a=7,c=8,h=9,d=10,u=11,l=12,f=13,g=14,_=function(e,t){for(var s in e)if(e.hasOwnProperty(s)){if(!t.hasOwnProperty(s)){var n="Unknown property, "+s+". Valid properties are:";for(var s in t)t.hasOwnProperty(s)&&(n=n+" "+s);throw new Error(n)}if(typeof e[s]!==t[s])throw new Error(v(w.INVALID_TYPE,[typeof e[s],s]))}},p=function(e,t){return function(){return e.apply(t,arguments)}},w={OK:{code:0,text:"AMQJSC0000I OK."},CONNECT_TIMEOUT:{code:1,text:"AMQJSC0001E Connect timed out."},SUBSCRIBE_TIMEOUT:{code:2,text:"AMQJS0002E Subscribe timed out."},UNSUBSCRIBE_TIMEOUT:{code:3,text:"AMQJS0003E Unsubscribe timed out."},PING_TIMEOUT:{code:4,text:"AMQJS0004E Ping timed out."},INTERNAL_ERROR:{code:5,text:"AMQJS0005E Internal error. Error Message: {0}, Stack trace: {1}"},CONNACK_RETURNCODE:{code:6,text:"AMQJS0006E Bad Connack return code:{0} {1}."},SOCKET_ERROR:{code:7,text:"AMQJS0007E Socket error:{0}."},SOCKET_CLOSE:{code:8,text:"AMQJS0008I Socket closed."},MALFORMED_UTF:{code:9,text:"AMQJS0009E Malformed UTF data:{0} {1} {2}."},UNSUPPORTED:{code:10,text:"AMQJS0010E {0} is not supported by this browser."},INVALID_STATE:{code:11,text:"AMQJS0011E Invalid state {0}."},INVALID_TYPE:{code:12,text:"AMQJS0012E Invalid type {0} for {1}."},INVALID_ARGUMENT:{code:13,text:"AMQJS0013E Invalid argument {0} for {1}."},UNSUPPORTED_OPERATION:{code:14,text:"AMQJS0014E Unsupported operation."},INVALID_STORED_DATA:{code:15,text:"AMQJS0015E Invalid data in local storage key={0} value={1}."},INVALID_MQTT_MESSAGE_TYPE:{code:16,text:"AMQJS0016E Invalid MQTT message type {0}."},MALFORMED_UNICODE:{code:17,text:"AMQJS0017E Malformed Unicode string:{0} {1}."}},y={0:"Connection Accepted",1:"Connection Refused: unacceptable protocol version",2:"Connection Refused: identifier rejected",3:"Connection Refused: server unavailable",4:"Connection Refused: bad user name or password",5:"Connection Refused: not authorized"},v=function(e,t){var s=e.text;if(t)for(var n,i,o=0;o<t.length;o++)if(n="{"+o+"}",(i=s.indexOf(n))>0){var r=s.substring(0,i),a=s.substring(i+n.length);s=r+t[o]+a}return s},I=[0,6,77,81,73,115,100,112,3],M=[0,4,77,81,84,84,4],m=function(e,t){for(var s in this.type=e,t)t.hasOwnProperty(s)&&(this[s]=t[s])};function E(e,t){var c,d=t,l=e[t],f=l>>4,g=l&=15;t+=1;var _=0,p=1;do{if(t==e.length)return[null,d];_+=(127&(c=e[t++]))*p,p*=128}while(0!=(128&c));var w=t+_;if(w>e.length)return[null,d];var y=new m(f);switch(f){case s:1&e[t++]&&(y.sessionPresent=!0),y.returnCode=e[t++];break;case n:var v=g>>1&3,I=O(e,t),M=C(e,t+=2,I);t+=I,v>0&&(y.messageIdentifier=O(e,t),t+=2);var E=new Paho.MQTT.Message(e.subarray(t,w));1==(1&g)&&(E.retained=!0),8==(8&g)&&(E.duplicate=!0),E.qos=v,E.destinationName=M,y.payloadMessage=E;break;case i:case o:case r:case a:case u:y.messageIdentifier=O(e,t);break;case h:y.messageIdentifier=O(e,t),t+=2,y.returnCode=e.subarray(t,w)}return[y,w]}function A(e,t,s){return t[s++]=e>>8,t[s++]=e%256,s}function T(e,t,s,n){return S(e,s,n=A(t,s,n)),n+t}function O(e,t){return 256*e[t]+e[t+1]}function N(e){for(var t=0,s=0;s<e.length;s++){var n=e.charCodeAt(s);n>2047?(55296<=n&&n<=56319&&(s++,t++),t+=3):n>127?t+=2:t++}return t}function S(e,t,s){for(var n=s,i=0;i<e.length;i++){var o=e.charCodeAt(i);if(55296<=o&&o<=56319){var r=e.charCodeAt(++i);if(isNaN(r))throw new Error(v(w.MALFORMED_UNICODE,[o,r]));o=r-56320+(o-55296<<10)+65536}o<=127?t[n++]=o:o<=2047?(t[n++]=o>>6&31|192,t[n++]=63&o|128):o<=65535?(t[n++]=o>>12&15|224,t[n++]=o>>6&63|128,t[n++]=63&o|128):(t[n++]=o>>18&7|240,t[n++]=o>>12&63|128,t[n++]=o>>6&63|128,t[n++]=63&o|128)}return t}function C(e,t,s){for(var n,i="",o=t;o<t+s;){var r=e[o++];if(r<128)n=r;else{var a=e[o++]-128;if(a<0)throw new Error(v(w.MALFORMED_UTF,[r.toString(16),a.toString(16),""]));if(r<224)n=64*(r-192)+a;else{var c=e[o++]-128;if(c<0)throw new Error(v(w.MALFORMED_UTF,[r.toString(16),a.toString(16),c.toString(16)]));if(r<240)n=4096*(r-224)+64*a+c;else{var h=e[o++]-128;if(h<0)throw new Error(v(w.MALFORMED_UTF,[r.toString(16),a.toString(16),c.toString(16),h.toString(16)]));if(!(r<248))throw new Error(v(w.MALFORMED_UTF,[r.toString(16),a.toString(16),c.toString(16),h.toString(16)]));n=262144*(r-240)+4096*a+64*c+h}}}n>65535&&(n-=65536,i+=String.fromCharCode(55296+(n>>10)),n=56320+(1023&n)),i+=String.fromCharCode(n)}return i}m.prototype.encode=function(){var e=(15&this.type)<<4,s=0,i=new Array,o=0;switch(null!=this.messageIdentifier&&(s+=2),this.type){case t:switch(this.mqttVersion){case 3:s+=I.length+3;break;case 4:s+=M.length+3}if(s+=N(this.clientId)+2,null!=this.willMessage){s+=N(this.willMessage.destinationName)+2;var a=this.willMessage.payloadBytes;a instanceof Uint8Array||(a=new Uint8Array(u)),s+=a.byteLength+2}null!=this.userName&&(s+=N(this.userName)+2),null!=this.password&&(s+=N(this.password)+2);break;case c:e|=2;for(var h=0;h<this.topics.length;h++)i[h]=N(this.topics[h]),s+=i[h]+2;s+=this.requestedQos.length;break;case d:e|=2;for(h=0;h<this.topics.length;h++)i[h]=N(this.topics[h]),s+=i[h]+2;break;case r:e|=2;break;case n:this.payloadMessage.duplicate&&(e|=8),e=e|=this.payloadMessage.qos<<1,this.payloadMessage.retained&&(e|=1),s+=(o=N(this.payloadMessage.destinationName))+2;var u=this.payloadMessage.payloadBytes;s+=u.byteLength,u instanceof ArrayBuffer?u=new Uint8Array(u):u instanceof Uint8Array||(u=new Uint8Array(u.buffer))}var l=function(e){var t=new Array(1),s=0;do{var n=e%128;(e>>=7)>0&&(n|=128),t[s++]=n}while(e>0&&s<4);return t}(s),f=l.length+1,g=new ArrayBuffer(s+f),_=new Uint8Array(g);if(_[0]=e,_.set(l,1),this.type==n)f=T(this.payloadMessage.destinationName,o,_,f);else if(this.type==t){switch(this.mqttVersion){case 3:_.set(I,f),f+=I.length;break;case 4:_.set(M,f),f+=M.length}var p=0;this.cleanSession&&(p=2),null!=this.willMessage&&(p|=4,p|=this.willMessage.qos<<3,this.willMessage.retained&&(p|=32)),null!=this.userName&&(p|=128),null!=this.password&&(p|=64),_[f++]=p,f=A(this.keepAliveInterval,_,f)}switch(null!=this.messageIdentifier&&(f=A(this.messageIdentifier,_,f)),this.type){case t:f=T(this.clientId,N(this.clientId),_,f),null!=this.willMessage&&(f=T(this.willMessage.destinationName,N(this.willMessage.destinationName),_,f),f=A(a.byteLength,_,f),_.set(a,f),f+=a.byteLength),null!=this.userName&&(f=T(this.userName,N(this.userName),_,f)),null!=this.password&&(f=T(this.password,N(this.password),_,f));break;case n:_.set(u,f);break;case c:for(h=0;h<this.topics.length;h++)f=T(this.topics[h],i[h],_,f),_[f++]=this.requestedQos[h];break;case d:for(h=0;h<this.topics.length;h++)f=T(this.topics[h],i[h],_,f)}return g};var R=function(e,t,s){this._client=e,this._window=t,this._keepAliveInterval=1e3*s,this.isReset=!1;var n=new m(l).encode(),i=function(e){return function(){return o.apply(e)}},o=function(){this.isReset?(this.isReset=!1,this._client._trace("Pinger.doPing","send PINGREQ"),this._client.socket.send(n),this.timeout=this._window.setTimeout(i(this),this._keepAliveInterval)):(this._client._trace("Pinger.doPing","Timed out"),this._client._disconnected(w.PING_TIMEOUT.code,v(w.PING_TIMEOUT)))};this.reset=function(){this.isReset=!0,this._window.clearTimeout(this.timeout),this._keepAliveInterval>0&&(this.timeout=setTimeout(i(this),this._keepAliveInterval))},this.cancel=function(){this._window.clearTimeout(this.timeout)}},b=function(e,t,s,n,i){this._window=t,s||(s=30);this.timeout=setTimeout(function(e,t,s){return function(){return e.apply(t,s)}}(n,e,i),1e3*s),this.cancel=function(){this._window.clearTimeout(this.timeout)}},k=function(t,s,n,i,o){if(!("WebSocket"in e)||null===e.WebSocket)throw new Error(v(w.UNSUPPORTED,["WebSocket"]));if(!("localStorage"in e)||null===e.localStorage)throw new Error(v(w.UNSUPPORTED,["localStorage"]));if(!("ArrayBuffer"in e)||null===e.ArrayBuffer)throw new Error(v(w.UNSUPPORTED,["ArrayBuffer"]));for(var r in this._trace("Paho.MQTT.Client",t,s,n,i,o),this.host=s,this.port=n,this.path=i,this.uri=t,this.clientId=o,this._localKey=s+":"+n+("/mqtt"!=i?":"+i:"")+":"+o+":",this._msg_queue=[],this._sentMessages={},this._receivedMessages={},this._notify_msg_sent={},this._message_identifier=1,this._sequence=0,localStorage)0!=r.indexOf("Sent:"+this._localKey)&&0!=r.indexOf("Received:"+this._localKey)||this.restore(r)};k.prototype.host,k.prototype.port,k.prototype.path,k.prototype.uri,k.prototype.clientId,k.prototype.socket,k.prototype.connected=!1,k.prototype.maxMessageIdentifier=65536,k.prototype.connectOptions,k.prototype.hostIndex,k.prototype.onConnectionLost,k.prototype.onMessageDelivered,k.prototype.onMessageArrived,k.prototype.traceFunction,k.prototype._msg_queue=null,k.prototype._connectTimeout,k.prototype.sendPinger=null,k.prototype.receivePinger=null,k.prototype.receiveBuffer=null,k.prototype._traceBuffer=null,k.prototype._MAX_TRACE_ENTRIES=100,k.prototype.connect=function(e){var t=this._traceMask(e,"password");if(this._trace("Client.connect",t,this.socket,this.connected),this.connected)throw new Error(v(w.INVALID_STATE,["already connected"]));if(this.socket)throw new Error(v(w.INVALID_STATE,["already connected"]));this.connectOptions=e,e.uris?(this.hostIndex=0,this._doConnect(e.uris[0])):this._doConnect(this.uri)},k.prototype.subscribe=function(e,t){if(this._trace("Client.subscribe",e,t),!this.connected)throw new Error(v(w.INVALID_STATE,["not connected"]));var s=new m(c);s.topics=[e],null!=t.qos?s.requestedQos=[t.qos]:s.requestedQos=[0],t.onSuccess&&(s.onSuccess=function(e){t.onSuccess({invocationContext:t.invocationContext,grantedQos:e})}),t.onFailure&&(s.onFailure=function(e){t.onFailure({invocationContext:t.invocationContext,errorCode:e})}),t.timeout&&(s.timeOut=new b(this,window,t.timeout,t.onFailure,[{invocationContext:t.invocationContext,errorCode:w.SUBSCRIBE_TIMEOUT.code,errorMessage:v(w.SUBSCRIBE_TIMEOUT)}])),this._requires_ack(s),this._schedule_message(s)},k.prototype.unsubscribe=function(e,t){if(this._trace("Client.unsubscribe",e,t),!this.connected)throw new Error(v(w.INVALID_STATE,["not connected"]));var s=new m(d);s.topics=[e],t.onSuccess&&(s.callback=function(){t.onSuccess({invocationContext:t.invocationContext})}),t.timeout&&(s.timeOut=new b(this,window,t.timeout,t.onFailure,[{invocationContext:t.invocationContext,errorCode:w.UNSUBSCRIBE_TIMEOUT.code,errorMessage:v(w.UNSUBSCRIBE_TIMEOUT)}])),this._requires_ack(s),this._schedule_message(s)},k.prototype.send=function(e){if(this._trace("Client.send",e),!this.connected)throw new Error(v(w.INVALID_STATE,["not connected"]));wireMessage=new m(n),wireMessage.payloadMessage=e,e.qos>0?this._requires_ack(wireMessage):this.onMessageDelivered&&(this._notify_msg_sent[wireMessage]=this.onMessageDelivered(wireMessage.payloadMessage)),this._schedule_message(wireMessage)},k.prototype.disconnect=function(){if(this._trace("Client.disconnect"),!this.socket)throw new Error(v(w.INVALID_STATE,["not connecting or connected"]));wireMessage=new m(g),this._notify_msg_sent[wireMessage]=p(this._disconnected,this),this._schedule_message(wireMessage)},k.prototype.getTraceLog=function(){if(null!==this._traceBuffer){for(var e in this._trace("Client.getTraceLog",new Date),this._trace("Client.getTraceLog in flight messages",this._sentMessages.length),this._sentMessages)this._trace("_sentMessages ",e,this._sentMessages[e]);for(var e in this._receivedMessages)this._trace("_receivedMessages ",e,this._receivedMessages[e]);return this._traceBuffer}},k.prototype.startTrace=function(){null===this._traceBuffer&&(this._traceBuffer=[]),this._trace("Client.startTrace",new Date,"@VERSION@")},k.prototype.stopTrace=function(){delete this._traceBuffer},k.prototype._doConnect=function(e){if(this.connectOptions.useSSL){var t=e.split(":");t[0]="wss",e=t.join(":")}this.connected=!1,this.connectOptions.mqttVersion<4?this.socket=new WebSocket(e,["mqttv3.1"]):this.socket=new WebSocket(e,["mqtt"]),this.socket.binaryType="arraybuffer",this.socket.onopen=p(this._on_socket_open,this),this.socket.onmessage=p(this._on_socket_message,this),this.socket.onerror=p(this._on_socket_error,this),this.socket.onclose=p(this._on_socket_close,this),this.sendPinger=new R(this,window,this.connectOptions.keepAliveInterval),this.receivePinger=new R(this,window,this.connectOptions.keepAliveInterval),this._connectTimeout=new b(this,window,this.connectOptions.timeout,this._disconnected,[w.CONNECT_TIMEOUT.code,v(w.CONNECT_TIMEOUT)])},k.prototype._schedule_message=function(e){this._msg_queue.push(e),this.connected&&this._process_queue()},k.prototype.store=function(e,t){var s={type:t.type,messageIdentifier:t.messageIdentifier,version:1};if(t.type!==n)throw Error(v(w.INVALID_STORED_DATA,[key,s]));t.pubRecReceived&&(s.pubRecReceived=!0),s.payloadMessage={};for(var i="",o=t.payloadMessage.payloadBytes,r=0;r<o.length;r++)o[r]<=15?i=i+"0"+o[r].toString(16):i+=o[r].toString(16);s.payloadMessage.payloadHex=i,s.payloadMessage.qos=t.payloadMessage.qos,s.payloadMessage.destinationName=t.payloadMessage.destinationName,t.payloadMessage.duplicate&&(s.payloadMessage.duplicate=!0),t.payloadMessage.retained&&(s.payloadMessage.retained=!0),0==e.indexOf("Sent:")&&(void 0===t.sequence&&(t.sequence=++this._sequence),s.sequence=t.sequence),localStorage.setItem(e+this._localKey+t.messageIdentifier,JSON.stringify(s))},k.prototype.restore=function(e){var t=localStorage.getItem(e),s=JSON.parse(t),i=new m(s.type,s);if(s.type!==n)throw Error(v(w.INVALID_STORED_DATA,[e,t]));for(var o=s.payloadMessage.payloadHex,r=new ArrayBuffer(o.length/2),a=new Uint8Array(r),c=0;o.length>=2;){var h=parseInt(o.substring(0,2),16);o=o.substring(2,o.length),a[c++]=h}var d=new Paho.MQTT.Message(a);d.qos=s.payloadMessage.qos,d.destinationName=s.payloadMessage.destinationName,s.payloadMessage.duplicate&&(d.duplicate=!0),s.payloadMessage.retained&&(d.retained=!0),i.payloadMessage=d,0==e.indexOf("Sent:"+this._localKey)?(i.payloadMessage.duplicate=!0,this._sentMessages[i.messageIdentifier]=i):0==e.indexOf("Received:"+this._localKey)&&(this._receivedMessages[i.messageIdentifier]=i)},k.prototype._process_queue=function(){for(var e=null,t=this._msg_queue.reverse();e=t.pop();)this._socket_send(e),this._notify_msg_sent[e]&&(this._notify_msg_sent[e](),delete this._notify_msg_sent[e])},k.prototype._requires_ack=function(e){var t=Object.keys(this._sentMessages).length;if(t>this.maxMessageIdentifier)throw Error("Too many messages:"+t);for(;void 0!==this._sentMessages[this._message_identifier];)this._message_identifier++;e.messageIdentifier=this._message_identifier,this._sentMessages[e.messageIdentifier]=e,e.type===n&&this.store("Sent:",e),this._message_identifier===this.maxMessageIdentifier&&(this._message_identifier=1)},k.prototype._on_socket_open=function(){var e=new m(t,this.connectOptions);e.clientId=this.clientId,this._socket_send(e)},k.prototype._on_socket_message=function(e){this._trace("Client._on_socket_message",e.data);for(var t=this._deframeMessages(e.data),s=0;s<t.length;s+=1)this._handleMessage(t[s])},k.prototype._deframeMessages=function(e){var t=new Uint8Array(e);if(this.receiveBuffer){var s=new Uint8Array(this.receiveBuffer.length+t.length);s.set(this.receiveBuffer),s.set(t,this.receiveBuffer.length),t=s,delete this.receiveBuffer}try{for(var n=0,i=[];n<t.length;){var o=E(t,n),r=o[0];if(n=o[1],null===r)break;i.push(r)}n<t.length&&(this.receiveBuffer=t.subarray(n))}catch(e){return void this._disconnected(w.INTERNAL_ERROR.code,v(w.INTERNAL_ERROR,[e.message,e.stack.toString()]))}return i},k.prototype._handleMessage=function(e){this._trace("Client._handleMessage",e);try{switch(e.type){case s:if(this._connectTimeout.cancel(),this.connectOptions.cleanSession){for(var t in this._sentMessages){var c=this._sentMessages[t];localStorage.removeItem("Sent:"+this._localKey+c.messageIdentifier)}for(var t in this._sentMessages={},this._receivedMessages){var d=this._receivedMessages[t];localStorage.removeItem("Received:"+this._localKey+d.messageIdentifier)}this._receivedMessages={}}if(0!==e.returnCode){this._disconnected(w.CONNACK_RETURNCODE.code,v(w.CONNACK_RETURNCODE,[e.returnCode,y[e.returnCode]]));break}this.connected=!0,this.connectOptions.uris&&(this.hostIndex=this.connectOptions.uris.length);var l=new Array;for(var g in this._sentMessages)this._sentMessages.hasOwnProperty(g)&&l.push(this._sentMessages[g]);l=l.sort((function(e,t){return e.sequence-t.sequence}));for(var _=0,p=l.length;_<p;_++){if((c=l[_]).type==n&&c.pubRecReceived){var I=new m(r,{messageIdentifier:c.messageIdentifier});this._schedule_message(I)}else this._schedule_message(c)}this.connectOptions.onSuccess&&this.connectOptions.onSuccess({invocationContext:this.connectOptions.invocationContext}),this._process_queue();break;case n:this._receivePublish(e);break;case i:(c=this._sentMessages[e.messageIdentifier])&&(delete this._sentMessages[e.messageIdentifier],localStorage.removeItem("Sent:"+this._localKey+e.messageIdentifier),this.onMessageDelivered&&this.onMessageDelivered(c.payloadMessage));break;case o:if(c=this._sentMessages[e.messageIdentifier]){c.pubRecReceived=!0;I=new m(r,{messageIdentifier:e.messageIdentifier});this.store("Sent:",c),this._schedule_message(I)}break;case r:d=this._receivedMessages[e.messageIdentifier];localStorage.removeItem("Received:"+this._localKey+e.messageIdentifier),d&&(this._receiveMessage(d),delete this._receivedMessages[e.messageIdentifier]);var M=new m(a,{messageIdentifier:e.messageIdentifier});this._schedule_message(M);break;case a:c=this._sentMessages[e.messageIdentifier];delete this._sentMessages[e.messageIdentifier],localStorage.removeItem("Sent:"+this._localKey+e.messageIdentifier),this.onMessageDelivered&&this.onMessageDelivered(c.payloadMessage);break;case h:(c=this._sentMessages[e.messageIdentifier])&&(c.timeOut&&c.timeOut.cancel(),128===e.returnCode[0]?c.onFailure&&c.onFailure(e.returnCode):c.onSuccess&&c.onSuccess(e.returnCode),delete this._sentMessages[e.messageIdentifier]);break;case u:(c=this._sentMessages[e.messageIdentifier])&&(c.timeOut&&c.timeOut.cancel(),c.callback&&c.callback(),delete this._sentMessages[e.messageIdentifier]);break;case f:this.sendPinger.reset();break;default:this._disconnected(w.INVALID_MQTT_MESSAGE_TYPE.code,v(w.INVALID_MQTT_MESSAGE_TYPE,[e.type]))}}catch(e){return void this._disconnected(w.INTERNAL_ERROR.code,v(w.INTERNAL_ERROR,[e.message,e.stack.toString()]))}},k.prototype._on_socket_error=function(e){this._disconnected(w.SOCKET_ERROR.code,v(w.SOCKET_ERROR,[e.data]))},k.prototype._on_socket_close=function(){this._disconnected(w.SOCKET_CLOSE.code,v(w.SOCKET_CLOSE))},k.prototype._socket_send=function(e){if(1==e.type){var t=this._traceMask(e,"password");this._trace("Client._socket_send",t)}else this._trace("Client._socket_send",e);this.socket.send(e.encode()),this.sendPinger.reset()},k.prototype._receivePublish=function(e){switch(e.payloadMessage.qos){case"undefined":case 0:this._receiveMessage(e);break;case 1:var t=new m(i,{messageIdentifier:e.messageIdentifier});this._schedule_message(t),this._receiveMessage(e);break;case 2:this._receivedMessages[e.messageIdentifier]=e,this.store("Received:",e);var s=new m(o,{messageIdentifier:e.messageIdentifier});this._schedule_message(s);break;default:throw Error("Invaild qos="+wireMmessage.payloadMessage.qos)}},k.prototype._receiveMessage=function(e){this.onMessageArrived&&this.onMessageArrived(e.payloadMessage)},k.prototype._disconnected=function(e,t){this._trace("Client._disconnected",e,t),this.sendPinger.cancel(),this.receivePinger.cancel(),this._connectTimeout&&this._connectTimeout.cancel(),this._msg_queue=[],this._notify_msg_sent={},this.socket&&(this.socket.onopen=null,this.socket.onmessage=null,this.socket.onerror=null,this.socket.onclose=null,1===this.socket.readyState&&this.socket.close(),delete this.socket),this.connectOptions.uris&&this.hostIndex<this.connectOptions.uris.length-1?(this.hostIndex++,this._doConnect(this.connectOptions.uris[this.hostIndex])):(void 0===e&&(e=w.OK.code,t=v(w.OK)),this.connected?(this.connected=!1,this.onConnectionLost&&this.onConnectionLost({errorCode:e,errorMessage:t})):4===this.connectOptions.mqttVersion&&!1===this.connectOptions.mqttVersionExplicit?(this._trace("Failed to connect V4, dropping back to V3"),this.connectOptions.mqttVersion=3,this.connectOptions.uris?(this.hostIndex=0,this._doConnect(this.connectOptions.uris[0])):this._doConnect(this.uri)):this.connectOptions.onFailure&&this.connectOptions.onFailure({invocationContext:this.connectOptions.invocationContext,errorCode:e,errorMessage:t}))},k.prototype._trace=function(){if(this.traceFunction){for(var e in arguments)void 0!==arguments[e]&&(arguments[e]=JSON.stringify(arguments[e]));var t=Array.prototype.slice.call(arguments).join("");this.traceFunction({severity:"Debug",message:t})}if(null!==this._traceBuffer){e=0;for(var s=arguments.length;e<s;e++)this._traceBuffer.length==this._MAX_TRACE_ENTRIES&&this._traceBuffer.shift(),0===e||void 0===arguments[e]?this._traceBuffer.push(arguments[e]):this._traceBuffer.push("  "+JSON.stringify(arguments[e]))}},k.prototype._traceMask=function(e,t){var s={};for(var n in e)e.hasOwnProperty(n)&&(s[n]=n==t?"******":e[n]);return s};var D=function(e,t,s,n){var i;if("string"!=typeof e)throw new Error(v(w.INVALID_TYPE,[typeof e,"host"]));if(2==arguments.length){n=t;var o=(i=e).match(/^(wss?):\/\/((\[(.+)\])|([^\/]+?))(:(\d+))?(\/.*)$/);if(!o)throw new Error(v(w.INVALID_ARGUMENT,[e,"host"]));e=o[4]||o[2],t=parseInt(o[7]),s=o[8]}else{if(3==arguments.length&&(n=s,s="/mqtt"),"number"!=typeof t||t<0)throw new Error(v(w.INVALID_TYPE,[typeof t,"port"]));if("string"!=typeof s)throw new Error(v(w.INVALID_TYPE,[typeof s,"path"]));var r=-1!=e.indexOf(":")&&"["!=e.slice(0,1)&&"]"!=e.slice(-1);i="ws://"+(r?"["+e+"]":e)+":"+t+s}for(var a=0,c=0;c<n.length;c++){var h=n.charCodeAt(c);55296<=h&&h<=56319&&c++,a++}if("string"!=typeof n||a>65535)throw new Error(v(w.INVALID_ARGUMENT,[n,"clientId"]));var d=new k(i,e,t,s,n);this._getHost=function(){return e},this._setHost=function(){throw new Error(v(w.UNSUPPORTED_OPERATION))},this._getPort=function(){return t},this._setPort=function(){throw new Error(v(w.UNSUPPORTED_OPERATION))},this._getPath=function(){return s},this._setPath=function(){throw new Error(v(w.UNSUPPORTED_OPERATION))},this._getURI=function(){return i},this._setURI=function(){throw new Error(v(w.UNSUPPORTED_OPERATION))},this._getClientId=function(){return d.clientId},this._setClientId=function(){throw new Error(v(w.UNSUPPORTED_OPERATION))},this._getOnConnectionLost=function(){return d.onConnectionLost},this._setOnConnectionLost=function(e){if("function"!=typeof e)throw new Error(v(w.INVALID_TYPE,[typeof e,"onConnectionLost"]));d.onConnectionLost=e},this._getOnMessageDelivered=function(){return d.onMessageDelivered},this._setOnMessageDelivered=function(e){if("function"!=typeof e)throw new Error(v(w.INVALID_TYPE,[typeof e,"onMessageDelivered"]));d.onMessageDelivered=e},this._getOnMessageArrived=function(){return d.onMessageArrived},this._setOnMessageArrived=function(e){if("function"!=typeof e)throw new Error(v(w.INVALID_TYPE,[typeof e,"onMessageArrived"]));d.onMessageArrived=e},this._getTrace=function(){return d.traceFunction},this._setTrace=function(e){if("function"!=typeof e)throw new Error(v(w.INVALID_TYPE,[typeof e,"onTrace"]));d.traceFunction=e},this.connect=function(e){if(_(e=e||{},{timeout:"number",userName:"string",password:"string",willMessage:"object",keepAliveInterval:"number",cleanSession:"boolean",useSSL:"boolean",invocationContext:"object",onSuccess:"function",onFailure:"function",hosts:"object",ports:"object",mqttVersion:"number",mqttVersionExplicit:"boolean",uris:"object"}),void 0===e.keepAliveInterval&&(e.keepAliveInterval=60),e.mqttVersion>4||e.mqttVersion<3)throw new Error(v(w.INVALID_ARGUMENT,[e.mqttVersion,"connectOptions.mqttVersion"]));if(void 0===e.mqttVersion?(e.mqttVersionExplicit=!1,e.mqttVersion=4):e.mqttVersionExplicit=!0,void 0!==e.password&&void 0===e.userName)throw new Error(v(w.INVALID_ARGUMENT,[e.password,"connectOptions.password"]));if(e.willMessage){if(!(e.willMessage instanceof P))throw new Error(v(w.INVALID_TYPE,[e.willMessage,"connectOptions.willMessage"]));if(e.willMessage.stringPayload,void 0===e.willMessage.destinationName)throw new Error(v(w.INVALID_TYPE,[typeof e.willMessage.destinationName,"connectOptions.willMessage.destinationName"]))}if(void 0===e.cleanSession&&(e.cleanSession=!0),e.hosts){if(!(e.hosts instanceof Array))throw new Error(v(w.INVALID_ARGUMENT,[e.hosts,"connectOptions.hosts"]));if(e.hosts.length<1)throw new Error(v(w.INVALID_ARGUMENT,[e.hosts,"connectOptions.hosts"]));for(var t=!1,n=0;n<e.hosts.length;n++){if("string"!=typeof e.hosts[n])throw new Error(v(w.INVALID_TYPE,[typeof e.hosts[n],"connectOptions.hosts["+n+"]"]));if(/^(wss?):\/\/((\[(.+)\])|([^\/]+?))(:(\d+))?(\/.*)$/.test(e.hosts[n])){if(0==n)t=!0;else if(!t)throw new Error(v(w.INVALID_ARGUMENT,[e.hosts[n],"connectOptions.hosts["+n+"]"]))}else if(t)throw new Error(v(w.INVALID_ARGUMENT,[e.hosts[n],"connectOptions.hosts["+n+"]"]))}if(t)e.uris=e.hosts;else{if(!e.ports)throw new Error(v(w.INVALID_ARGUMENT,[e.ports,"connectOptions.ports"]));if(!(e.ports instanceof Array))throw new Error(v(w.INVALID_ARGUMENT,[e.ports,"connectOptions.ports"]));if(e.hosts.length!=e.ports.length)throw new Error(v(w.INVALID_ARGUMENT,[e.ports,"connectOptions.ports"]));e.uris=[];for(n=0;n<e.hosts.length;n++){if("number"!=typeof e.ports[n]||e.ports[n]<0)throw new Error(v(w.INVALID_TYPE,[typeof e.ports[n],"connectOptions.ports["+n+"]"]));var o=e.hosts[n],r=e.ports[n],a=-1!=o.indexOf(":");i="ws://"+(a?"["+o+"]":o)+":"+r+s,e.uris.push(i)}}}d.connect(e)},this.subscribe=function(e,t){if("string"!=typeof e)throw new Error("Invalid argument:"+e);if(_(t=t||{},{qos:"number",invocationContext:"object",onSuccess:"function",onFailure:"function",timeout:"number"}),t.timeout&&!t.onFailure)throw new Error("subscribeOptions.timeout specified with no onFailure callback.");if(void 0!==t.qos&&0!==t.qos&&1!==t.qos&&2!==t.qos)throw new Error(v(w.INVALID_ARGUMENT,[t.qos,"subscribeOptions.qos"]));d.subscribe(e,t)},this.unsubscribe=function(e,t){if("string"!=typeof e)throw new Error("Invalid argument:"+e);if(_(t=t||{},{invocationContext:"object",onSuccess:"function",onFailure:"function",timeout:"number"}),t.timeout&&!t.onFailure)throw new Error("unsubscribeOptions.timeout specified with no onFailure callback.");d.unsubscribe(e,t)},this.send=function(e,t,s,n){var i;if(0==arguments.length)throw new Error("Invalid argument.length");if(1==arguments.length){if(!(e instanceof P)&&"string"!=typeof e)throw new Error("Invalid argument:"+typeof e);if(void 0===(i=e).destinationName)throw new Error(v(w.INVALID_ARGUMENT,[i.destinationName,"Message.destinationName"]));d.send(i)}else(i=new P(t)).destinationName=e,arguments.length>=3&&(i.qos=s),arguments.length>=4&&(i.retained=n),d.send(i)},this.disconnect=function(){d.disconnect()},this.getTraceLog=function(){return d.getTraceLog()},this.startTrace=function(){d.startTrace()},this.stopTrace=function(){d.stopTrace()},this.isConnected=function(){return d.connected}};D.prototype={get host(){return this._getHost()},set host(e){this._setHost(e)},get port(){return this._getPort()},set port(e){this._setPort(e)},get path(){return this._getPath()},set path(e){this._setPath(e)},get clientId(){return this._getClientId()},set clientId(e){this._setClientId(e)},get onConnectionLost(){return this._getOnConnectionLost()},set onConnectionLost(e){this._setOnConnectionLost(e)},get onMessageDelivered(){return this._getOnMessageDelivered()},set onMessageDelivered(e){this._setOnMessageDelivered(e)},get onMessageArrived(){return this._getOnMessageArrived()},set onMessageArrived(e){this._setOnMessageArrived(e)},get trace(){return this._getTrace()},set trace(e){this._setTrace(e)}};var P=function(e){var t;if(!("string"==typeof e||e instanceof ArrayBuffer||e instanceof Int8Array||e instanceof Uint8Array||e instanceof Int16Array||e instanceof Uint16Array||e instanceof Int32Array||e instanceof Uint32Array||e instanceof Float32Array||e instanceof Float64Array))throw v(w.INVALID_ARGUMENT,[e,"newPayload"]);t=e,this._getPayloadString=function(){return"string"==typeof t?t:C(t,0,t.length)},this._getPayloadBytes=function(){if("string"==typeof t){var e=new ArrayBuffer(N(t)),s=new Uint8Array(e);return S(t,s,0),s}return t};var s=void 0;this._getDestinationName=function(){return s},this._setDestinationName=function(e){if("string"!=typeof e)throw new Error(v(w.INVALID_ARGUMENT,[e,"newDestinationName"]));s=e};var n=0;this._getQos=function(){return n},this._setQos=function(e){if(0!==e&&1!==e&&2!==e)throw new Error("Invalid argument:"+e);n=e};var i=!1;this._getRetained=function(){return i},this._setRetained=function(e){if("boolean"!=typeof e)throw new Error(v(w.INVALID_ARGUMENT,[e,"newRetained"]));i=e};var o=!1;this._getDuplicate=function(){return o},this._setDuplicate=function(e){o=e}};return P.prototype={get payloadString(){return this._getPayloadString()},get payloadBytes(){return this._getPayloadBytes()},get destinationName(){return this._getDestinationName()},set destinationName(e){this._setDestinationName(e)},get qos(){return this._getQos()},set qos(e){this._setQos(e)},get retained(){return this._getRetained()},set retained(e){this._setRetained(e)},get duplicate(){return this._getDuplicate()},set duplicate(e){this._setDuplicate(e)}},{Client:D,Message:P}}(window);
)rawliteral";
#endif
//Done with USE_GRAPHER
///EOF///
