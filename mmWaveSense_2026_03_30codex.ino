
#include <Arduino.h>
#define CONFIG_ARDUINO_LOOP_STACK_SIZE 16 * 1024

#define ESP32POE
#define USEBASEMAC

// Switch to new style over-the-air updater
#define NEWELEGANTOTA
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1

#define DOHEALTHCHECK

#define USE_GRAPHER

// See https://wiki.seeedstudio.com/mmwave_human_detection_kit/#hardware-overview
//#ifndef RX1
//#define RX1 4
//#define TX1 5
//#endif


#ifndef RX1
#warning "Missing serial 1 pins!"
#error "Did you choose the right board type?"
#endif

// DEBUG MQTT compiles about 5% larger.
//#define ASYNC_MQTT_DEBUG_PORT Serial
//#define _ASYNC_MQTT_LOGLEVEL_ 4

// Note that changing the TAG will change the NVS data set name and clear all settings!!
#define TAG "mmW32"
#define SEPARATOR "_"

// This should be at most 8 characters
#define DEVICETYPE "mmWaver32"
// Using progmem keeps these constants out of RAM
const char DeviceType[] PROGMEM = DEVICETYPE;


// Misc
#include "mmWaveSense.h"

// initialized with a setUpdateInterval of 1 hour in millis
// SNTP client via UDP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTPSERVERNAME, 0, 3600000);


inline unsigned long GetUpMinutes() {
  return (unsigned long)millis() / 1000 / 60;
}



//
inline void feed_watchdog() {
  if (MustFeedWatchDog) {
    vTaskDelay(1);
#ifdef WDT_TIMEOUT
    if (xTaskGetCurrentTaskHandle() == WatchedTask) {
      log_d("wdt_task '%s' feeding watchdog", pcTaskGetName(WatchedTask));
      switch (esp_task_wdt_reset()) {
        case ESP_ERR_NOT_FOUND:
          log_e("wdt_task '%s' not subscribed to watchdog!", pcTaskGetName(xTaskGetCurrentTaskHandle()));
          break;
        case ESP_ERR_INVALID_STATE:
          log_d("Watchdog uninitialized!");
          break;
        case ESP_OK:
          LastFedWatchdog = TimeNow;
          delay(1);
      }
    }
#endif
  }
}

#ifdef NEWELEGANTOTA
////////////////////
void onOTAStart() {
  log_w("OTA update process started.");
  ProcessingOTA = true;

#ifdef LITELED
  if (UseLedStrip) {
    strip.clear(true);
    strip.brightness(0, true);
  }
#endif
}

////////////////////
void onOTAProgress(size_t current, size_t final) {
  static unsigned long ota_progress_millis = 0;
  // Log every X second
  if (millis() - ota_progress_millis > 4000) {
    ota_progress_millis = millis();
    feed_watchdog();
    Serial.print("@");
  }
}

////////////////////
void onOTAEnd(bool success) {
  ProcessingOTA = false;
  if (success) {
    log_w("OTA update completed successfully.");
  } else {
    log_e("OTA update failed!!!!");
    ProcessingOTA = false;
#ifdef LITELED
    if (UseLedStrip) {
      strip.clear(true);
      strip.brightness(0, true);
    }
#endif
  }
}
#endif


////////////////////
inline char *humanReadableTimeArray(time_t timestamp) {
  tm timeinfo = *localtime(&timestamp);
  strftime(TimeString, sizeof(TimeString) - 1, "%e %b %H:%M", &timeinfo);
  return (TimeString);
}

////////////////////
String humanReadableTime(time_t timestamp) {
  return (String(humanReadableTimeArray(timestamp)));
}



////////////////
boolean NetworkGood(boolean usecached) {
  //log_d("Checking IP");
  char *myip = FindMyIP();
  if (!ETH.linkUp()) {
    if (EthernetConnected > 0) log_w("Ethernet link went down");
    EthernetConnected = -444;
    return false;
  }
  if (ETH.linkUp()) {
    log_d("Got Ethernet link");
    if (0 == strcmp(zeroip, myip)) {
      EthernetConnected = -555;
      return false;
    }


    if (usecached) {
      log_d("IP address %s", myip);
      return EthernetConnected;
    } else log_w("IP address %s", myip);

    IPAddress test;
    feed_watchdog();  // tickle the watchdog

    log_d("Doing nslookup");
    int err = WiFi.hostByName(MQTTserver, test);
    if (1 == err) {
      EthernetConnected = true;
      WantWiFi = false;
      strcpy(MQTTserverAddress, test.toString().c_str());
      log_d("DNS Resolved '%s' to %s successfully", MQTTserver, MQTTserverAddress);
      log_d("did nslookup successfully");
#ifdef MQTT_SERVER_BACKUP
      if (!connectedMQTT && !strcmp(MQTT_SERVER, MQTTserver) && !strncmp(TESTNETPREFIX, myip, strlen(TESTNETPREFIX))) {
        log_w("Will use backup MQTT server %s", MQTT_SERVER_BACKUP);
        strcpy(MQTTserver, MQTT_SERVER_BACKUP);
      }
#endif
    } else {
      EthernetConnected = -999;
      return false;
    }
    return EthernetConnected;
  }

  return false;
}

////////////////
void onMqttConnect(bool sessionPresent) {
  char ident[URLBUFSIZE];
  log_d("-----------onMqttConnect---------------");
  connectedMQTT = true;
  connectToMqttTicker.detach();
  log_i("MQTT reports successfully connected to %s (%s).", MQTTserver, MQTTserverAddress);
  snprintf(ident, URLBUFSIZE - 1, "ESP32 %s %s %s %s connected.", NodeIDshort, __DATE__, __TIME__, __FILE__);
  ident[URLBUFSIZE - 1] = (char)0x00;
  mqttClient.publish(uniqueID(), 1, true, ident);
  uint16_t sub;

  sub = mqttClient.subscribe(Topic_Broadcast, 0);
  if (0 == sub) log_w("%s", MQTTBroadcastTSubFailed);

  if (!strcmp(MQTTReasonText, NOTYETDISCONNECTED))
    strncpy(MQTTReasonText, ConnSucc, sizeof(MQTTReasonText) - 1);

  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
  mqttClient.publish(MQTTtopic, 1, true, "");  // Purge retained
  if (INVALIDBODYSIGN != BodySignVal && BodySignVal && BodySignValCaptureTime < TimeNow - 67) {
    log_i("Reconnected, sending BodySignVal=%d", BodySignVal);
    FloorplanBodySign(BodySignVal, true);
  }
  if (!timeClient.isTimeSet()) {
    log_d("Setting clock");
    startNTP(false);
  }
}

////////////////
void connectToMqtt() {
  if (connectedMQTT) return;
  log_i("ReConnecting to MQTT...");
  if (!NetworkGood()) {
    log_i("No Network connection, will wait to retry");
    mqttReconnectTimer.once(MQTTRECONINTERVAL, connectToMqtt);  // in seconds
    return;
  }
  if (!connectedMQTT) {
    log_i("Requesting mqtt connection to %s:%d (%s)", MQTTserver, MQTT_PORT, MQTTserverAddress);
    startMQTT();
  }
}


////////////////
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  //(void)reason;
  static time_t lostConnectionTime = 0;

  if (!NetworkGood(false)) log_w("Network down");

  if (connectedMQTT) {
    mqttReconnectTimer.once(MQTTRECONINTERVAL, connectToMqtt);  // in seconds
    log_w("MQTT went Down, reconnecting MQTT");
    //log_w("Lost MQTT connection");
  }
  connectedMQTT = false;

  switch (reason) {
    case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
      strcpy(MQTTReasonText, "TCP Disconnect");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
      strcpy(MQTTReasonText, "Bad Protocol");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
      strcpy(MQTTReasonText, "Bad Identifier");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
      strcpy(MQTTReasonText, "No Server");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
      strcpy(MQTTReasonText, "Bad Creds");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
      strcpy(MQTTReasonText, "UnAuthorized");
      break;
    case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
      strcpy(MQTTReasonText, "Bad TLS");
      break;
    default:
      snprintf(MQTTReasonText, sizeof(MQTTReasonText) - 1, "Unknown %X", (unsigned int)reason);
      log_w("MQTT Disconnect cause is unknown value %X", reason);
      break;
  }
  log_d("MQTT Disconnect reason %s (%X)", MQTTReasonText, (unsigned int)reason);
  if (difftime(getNow(), lostConnectionTime) >= MQTTRECONINTERVAL) {
    log_w("MQTT disconnection %s", MQTTReasonText);
    lostConnectionTime = TimeNow;
  }
}

////////////////
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  log_d("Subscribe acknowledged, packetID %u, qos %u.", packetId, qos);
}


////////////////
// Automatically runs via ticker.
void connectToMqttCheck() {
  // Normally only called when not connected, so we need to establish a connection
  feed_watchdog();
  if (connectedMQTT) {
    log_d("-----------connectToMqttCheck OK, already connected");
    strncpy(MQTTReasonText, "Connected", sizeof(MQTTReasonText) - 1);
    return;  // Nothing needs doing
  }
  if (!NetworkGood()) {
    log_w("Network failure");
    strncpy(MQTTReasonText, "No Network", sizeof(MQTTReasonText) - 1);
    log_d("-----------connectToMqttCheck cannot proceed, no network!");
    return;
  }
  // To get this far, we have a network, but aren't connected to MQTT

  if (!connectedMQTT) {
    connectToMqtt();
    strncpy(MQTTReasonText, "Attempting reconnect via ticker", sizeof(MQTTReasonText) - 1);
    log_d("-----------connectToMqttCheck is calling connectToMqtt");
  }
}


////////////////
void onMqttMessage(const char *topic, const char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total) {
  (void)payload;

  log_i("MQTT Message received; topic = %s, len(%d)", topic, len);

  int offset = strlen(topic) - strlen(uniqueID());
  if (offset > 2) {
    if (0 == strcmp(topic + offset, uniqueID())) {
      log_i("Ignoring self-message in topic %s", topic);
      return;
    }
  }

  // To use the payload, copy len bytes, and then add a NULL byte at len+1
  if (0 == strcmp(topic, "msg/broadcast") && len < BUFFER_SIZE) {
    char BroadcastMessage[MSGBUFFER_SIZE];
    memcpy(BroadcastMessage, payload, len);
    BroadcastMessage[len] = (char)0x00;
    log_w("Received broadcast message %s", BroadcastMessage);
    return;

  } else log_i("Got message for topic %s", topic);
}


////////////////
void startMQTT() {
  if (connectedMQTT) {
    log_w("Already connected, nothing to do");
    return;
  }
  if (!NetworkGood()) {
    log_w("Still no network, cannot connect MQTT.  EC=%d", EthernetConnected);
    return;
  }

  IPAddress test;
  if (1 == WiFi.hostByName(MQTTserver, test)) {
    strcpy(MQTTserverAddress, test.toString().c_str());
    log_i("DNS Resolved '%s' to %s successfully", MQTTserver, MQTTserverAddress);
  } else strcpy(MQTTserverAddress, "Unresolved");

  log_w("MQTT initial connection to server %s:%d (%s)", MQTTserver, MQTT_PORT, MQTTserverAddress);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.setClientId(uniqueID());
  mqttClient.setCleanSession(false);
  mqttClient.setServer(MQTTserver, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  mqttClient.setKeepAlive(666);
  //mqttClient.setWill(uniqueID(), 1, true, "Dropped out");
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
  mqttClient.setWill(MQTTtopic, 1, true, "ghostwhite");  // Mark sensor as vacant on drop
  strncpy(MQTTReasonText, "Set Ticker in StartMQTT", sizeof(MQTTReasonText) - 1);
  log_i("connecting...");
  mqttClient.connect();
  log_i("MQTT will connect to server %s (%s)", MQTTserver, MQTTserverAddress);
  connectToMqttTicker.detach();
  connectToMqttTicker.attach_ms(MQTT_CHECK_INTERVAL_MS, connectToMqttCheck);
}



////////////////
void updateModes(boolean bootcall) {
  log_i("Setting modes");

  strcpy(Where, shortID(true));
  log_w("Setting Where and ShortID to '%s'", Where);


  log_i("Retrieving saved values from NVS");
  readNVS();  // Fetch saved values from NVS
  feed_watchdog();
  shortID(true);

#ifdef DOHEALTHCHECK
  if (!bootcall) RemoteHealthCallModify();
#endif

  log_d("Set modes");
  if (strlen(DisableSensor)) {
    WarnDisabled();
    if (!DisabledSensor) {
      DisabledSensor = true;
#ifdef DOHEALTHCHECK
      if (!bootcall) RemoteHealthAPI(HealthCheckPauseURL);
#endif
    }

  } else {
    log_w("Sensor Enabled");
    if (DisabledSensor) {
      DisabledSensor = false;
#ifdef DOHEALTHCHECK
      if (!bootcall) RemoteHealthAPI(HealthCheckResumeURL);
#endif
    }
  }
}

//////////////////////////////////////////
// Build "NoWhere" suffix
void build_nowhere() {
  NoWheres = SEPARATOR;
  if (strlen(Room) > 0) NoWheres.append(Room);
  else NoWheres.append(Where);
}

////////////////
// Fetch from onboard non-storage
void readNVS() {
  size_t bytes = 0;

  if (false == SavedNVS.begin(TAG, PREF_RO)) {
    log_e("failure to open NVS for %s", TAG);
    return;
  }
  log_d("Reading NVS");

#ifdef DOHEALTHCHECK
  bytes = SavedNVS.getString(nvs_healthpingurl, HealthCheckPingURL, (size_t)sizeof(HealthCheckPingURL));
  if (!bytes) bzero(HealthCheckPingURL, (size_t)sizeof(HealthCheckPingURL));
  bytes = SavedNVS.getString(nvs_healthurlpause, HealthCheckPauseURL, (size_t)sizeof(HealthCheckPauseURL));
  if (!bytes) bzero(HealthCheckPauseURL, (size_t)sizeof(HealthCheckPauseURL));
  bytes = SavedNVS.getString(nvs_healthresumeurl, HealthCheckResumeURL, (size_t)sizeof(HealthCheckResumeURL));
  if (!bytes) bzero(HealthCheckResumeURL, (size_t)sizeof(HealthCheckResumeURL));
  bytes = SavedNVS.getString(nvs_healthmodurl, HealthCheckUpdateURL, (size_t)sizeof(HealthCheckUpdateURL));
  if (!bytes) bzero(HealthCheckUpdateURL, (size_t)sizeof(HealthCheckUpdateURL));
#endif

  bytes = SavedNVS.getString(nvs_LastWebError, LastWebError, (size_t)ERRBUFSIZE - 1);
  if (!bytes) strcpy(LastWebError, "No saved error");

  bytes = SavedNVS.getString(nvs_whereword, Where, (size_t)BUFFER_SIZE - 1);
  if (!bytes) strcpy(Where, shortID());
  make_state_topic();  // Populate state_topic from current name

  bytes = SavedNVS.getString(nvs_MQTTServer, MQTTserver, (size_t)sizeof(MQTTserver) - 1);
  if (bytes) log_w("Populated MQTTServer from NVS with '%s'", MQTTserver);

  bytes = SavedNVS.getString(nvs_Floor, Floor, (size_t)BUFFER_SIZE - 1);
  bytes = SavedNVS.getString(nvs_Room, Room, (size_t)BUFFER_SIZE - 1);
  build_nowhere();

  bytes = SavedNVS.getString(nvs_DisableSensor, DisableSensor, (size_t)BUFFER_SIZE - 1);
  if (bytes && strlen(DisableSensor)) {
    DisabledSensor = true;
  } else DisabledSensor = false;

  SavedNVS.end();
}


////////////////
// Note that NVS has a hardcoded maximum key length of 15 bytes.
inline void writeNVS() {
  char buffer[MSGBUFFER_SIZE] = "Empty";
  size_t bytes;

  if (false == SavedNVS.begin(TAG, PREF_RW)) {
    log_e("failure to open NVS for '%s'", TAG);
    return;
  } else log_d("Writing to NVS as '%s'", TAG);


#ifdef DOHEALTHCHECK
  log_d("Saving HealthCheckPingURL");
  if (strlen(HealthCheckPingURL) > 5) {
    bytes = SavedNVS.getString(nvs_healthpingurl, buffer, (size_t)sizeof(HealthCheckPingURL));
    if (!bytes || (bytes && strncmp(HealthCheckPingURL, buffer, (size_t)sizeof(HealthCheckPingURL) - 1)))
      SavedNVS.putString(nvs_healthpingurl, HealthCheckPingURL);
    log_d("Saved healthurl value of '%s'", HealthCheckPingURL);
  }

  log_d("Saving HealthCheckUpdateURL");
  if (strlen(HealthCheckUpdateURL) > 5) {
    bytes = SavedNVS.getString(nvs_healthmodurl, buffer, (size_t)sizeof(HealthCheckUpdateURL));
    if (!bytes || (bytes && strncmp(HealthCheckUpdateURL, buffer, (size_t)sizeof(HealthCheckUpdateURL) - 1)))
      SavedNVS.putString(nvs_healthmodurl, HealthCheckUpdateURL);
    log_d("Saved healthurl modify value of '%s'", HealthCheckUpdateURL);
  }

  log_d("Saving HealthCheckPauseURL");
  if (strlen(HealthCheckPauseURL) > 5) {
    bytes = SavedNVS.getString(nvs_healthurlpause, buffer, (size_t)sizeof(HealthCheckPauseURL));
    if (!bytes || (bytes && strncmp(HealthCheckPauseURL, buffer, (size_t)sizeof(HealthCheckPauseURL) - 1)))
      SavedNVS.putString(nvs_healthurlpause, HealthCheckPauseURL);
    log_d("Saved healthurl pause value of '%s'", HealthCheckPauseURL);
  }

  log_d("Saving HealthCheckResumeURL");
  if (strlen(HealthCheckResumeURL) > 5) {
    bytes = SavedNVS.getString(nvs_healthresumeurl, buffer, (size_t)sizeof(HealthCheckResumeURL));
    if (!bytes || (bytes && strncmp(HealthCheckResumeURL, buffer, (size_t)sizeof(HealthCheckResumeURL) - 1)))
      SavedNVS.putString(nvs_healthresumeurl, HealthCheckResumeURL);
    log_d("Saved healthurl resume value of '%s'", HealthCheckResumeURL);
  }
#endif

  log_d("Updating Floor");
  if (strlen(Floor)) {
    bytes = SavedNVS.getString(nvs_Floor, buffer, (size_t)sizeof(Floor) - 1);
    if (!bytes || (bytes && strncmp(Floor, buffer, (size_t)sizeof(Floor) - 1)))
      SavedNVS.putString(nvs_Floor, Floor);
  } else if (SavedNVS.isKey(nvs_Floor)) SavedNVS.remove(nvs_Floor);

  log_d("Updating DisableSensor");
  if (strlen(DisableSensor)) {
    bytes = SavedNVS.getString(nvs_DisableSensor, buffer, (size_t)sizeof(DisableSensor) - 1);
    if (!bytes || (bytes && strncmp(DisableSensor, buffer, (size_t)sizeof(DisableSensor) - 1)))
      SavedNVS.putString(nvs_DisableSensor, DisableSensor);
    DisabledSensor = true;
  } else {
    if (SavedNVS.isKey(nvs_DisableSensor)) SavedNVS.remove(nvs_DisableSensor);
    DisabledSensor = false;
  }

  log_d("Updating Room");
  if (strlen(Room)) {
    bytes = SavedNVS.getString(nvs_Room, buffer, (size_t)sizeof(Room) - 1);
    if (!bytes || (bytes && strncmp(Room, buffer, (size_t)sizeof(Room) - 1)))
      SavedNVS.putString(nvs_Room, Room);
  } else if (SavedNVS.isKey(nvs_Room)) SavedNVS.remove(nvs_Room);
  build_nowhere();

  log_d("Updating Where");
  if (strlen(Where) && strcmp(Where, TAG)) {
    bytes = SavedNVS.getString(nvs_whereword, buffer, (size_t)sizeof(Where) - 1);
    if (!bytes || (bytes && strncmp(Where, buffer, (size_t)sizeof(Where) - 1)))
      SavedNVS.putString(nvs_whereword, Where);
  }
  make_state_topic();  // Populate state_topic from current name
  log_d("Closing NVS");
  SavedNVS.end();
  log_i("Closed NVS");
}


////////////////
void writeLastWebErrorToNVS(int errorcode) {
  if (false == SavedNVS.begin(TAG, PREF_RW)) {
    log_e("failure to open NVS for %s", TAG);
    return;
  }
  char buffer[ERRBUFSIZE] = "NoKnownWebError";
  size_t bytes;

  // Is the variable empty?
  if (!strlen(LastWebError) || 0 == strcmp(DEFAULTLASTWEBERRORSTRING, LastWebError))
    return;

  // Check if an identical value is already stored before writing
  bytes = SavedNVS.getString(nvs_LastWebError, buffer, (size_t)ERRBUFSIZE - 1);
  if (!bytes || (bytes && strncmp(LastWebError, buffer, (size_t)ERRBUFSIZE - 1))) {
    SavedNVS.putString(nvs_LastWebError, LastWebError);
    SavedNVS.putInt("LastWebEcode", (int32_t)errorcode);
  }

  if (SavedNVS.isKey(nvs_LastWebError)) log_d("+++ Key 'LWE' exists in NVS");
  else log_w("---Key 'LWE' still does not exist in NVS");

  SavedNVS.end();
}


////////////////
inline void do_delay(unsigned int ms) {
#ifndef WDT_TIMEOUT
  delay(ms);
#else
  feed_watchdog();
  // Delay at most 90% of the watchdog timeout interval.
  delay((((ms) < (900 * WDT_TIMEOUT)) ? (ms) : (900 * WDT_TIMEOUT)));
  feed_watchdog();
#endif
}


////////////////////
inline void handleReportHeapStack() {
  LastPostHeapTimeSeconds = TimeNow;
  //long LastHeapMaxAllocValue = (long)ESP.getMaxAllocHeap();
  snprintf(RamDataBuffer, sizeof(RamDataBuffer) - 1, "ram,tag=free,IP=\"%s\",room=\"%s\",where=\"%s\" stack=%u,heap=%lu,BodySign=%d,MQTT=%d",
           MyIPAddress, Room, Where,
           (unsigned int)uxTaskGetStackHighWaterMark(NULL), (unsigned long)ESP.getFreeHeap(), BodySignVal,
           (int)connectedMQTT);

  if (connectedMQTT) {
    snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/memory", Where);
    mqttClient.publish(MQTTtopic, 0, false, RamDataBuffer);
  }
  log_i("%s", RamDataBuffer);

  return;  // Skip report routine to debug memory leak

#ifdef DOHEALTHCHECK
  RemoteHealthCall(1, RamDataBuffer, FindMyIP());
#endif
  PostHeapIOTPlotter();
}

void setDNSpublic() {
#ifdef DNSOVERRIDE
  IPAddress google(8, 8, 8, 8);
  IPAddress cloudflare(1, 1, 1, 1);


  if (0 == strcmp(WiFi.dnsIP().toString().c_str(), zeroip)) {
    log_w("Setting WiFi DNS to public DNS servers");
    //WiFi.setDNS(google, cloudflare);
    //dns_setserver(0,google);
    //dns_setserver(1,cloudflare);
  } else if (ETH.linkUp()) {
    log_w("Setting Ethernet DNS to public DNS server");
    //ETH.setDnsServerIP(google);
  }
#endif
}




////////////////
Stream *testSerial(Stream *s) {
  s->print("Test");
  return (s);
}


////////////////
void setup() {
  Serial.begin(BAUD);  // open communication

  //#if defined(XIAO_ESP32C3) && defined(WDT_TIMEOUT)
  //init_watchdog(2 * WDT_TIMEOUT, true);  // Extend the timeout on single-core device
  //#endif
  //#endif

  // Wait on serial port
  //for (int ser = 0; ser < 20 && !Serial; ser++) vTaskDelay(100 / portTICK_PERIOD_MS);
  if (!Serial) delay(400);

  Serial.setDebugOutput(true);  // Allow log_ macros to send here
  Serial.write("==============================================================================");
  if (!Serial) do_delay(200);
  Serial.println("==============================================================================");
  do_delay(200);
  log_w("\n\n\n\n\nBoot.");
  shortID(true);  // Populate NodeIDshort
  log_w("%s v %s running %s, compiled %s %s", NodeIDshort, VERSION, __FILE__, __DATE__, __TIME__);

#ifdef ARDUINO_BOARD
  log_w("Arduino Board '%s', IDF Version %s", ARDUINO_BOARD, IDF_VER);
#else
  log_w("IDF Version %s", IDF_VER);
#endif
#ifdef ARDUINO_VARIANT
  log_w("Arduino '%s'", ARDUINO_VARIANT);
#endif
  do_delay(SleepDuration);
  log_e("Initial Setup - %s (%s) %s %s %s connected.", Where, MDNS_NAME, __DATE__, __TIME__, __FILE__);

  esp_reset_reason_t reason = esp_reset_reason();
  log_w("Booted due to reason %X.", reason);
  auto lastbootreason = logbootreason();

  StartHumanStaticPresenceLite();
  StartEthernet();

  SetRadarMode();

#ifdef WDT_TIMEOUT
  log_w("Will enable watchdog for %d seconds", WDT_TIMEOUT);
#endif

  logram();
  log_d("Retrieving saved values from NVS");
  readNVS();

  log_i("Starting Networking...");
  if (!ConnectNetwork()) {
    log_e("failed to Connect Network");
    do_delay(SleepDuration);
  } else {
#ifdef USE_MDNS
    const char *hn = hostName();
    boolean mdns_success = MDNS.begin(hn);
    if (!mdns_success) {
      delay(SleepDuration);  // Wait a bit and try again
      mdns_success = MDNS.begin(hn);
    }
    if (mdns_success) log_w("MDNS responder started as %s", hn);
    else log_e("MDNS.begin failure!");
#endif
  }

  log_d("setup web server");
  setupwebserver(HTTP_USER, HTTP_PASS);

  setenv("TZ", TZ, 1);
  tzset();
  log_d("NTP");
  startNTP(true);
  log_d("srand");
  srand((unsigned int)(millis() % 67296));

  if (ESP_RST_PANIC == lastbootreason) {
    log_w("IP: %s", getMyIP(true));
    log_e("Sleeping for OTA work after panic reboot");
    ElegantOTA.loop();
    for (unsigned int i = 0; i < 40; i++) {
      delay(500);
      ElegantOTA.loop();
    }
    log_e("Done sleeping after panic reboot");
    if (ProcessingOTA) {
      log_w("Processing OTA...");
      for (unsigned int i = 0; ProcessingOTA && i < 999; i++) {
        delay(100);
        ElegantOTA.loop();
      }
    }
  }

  // Initialize timestamps
  TimeNow = getNow();
  if (goodClock()) log_i("Current time is %llu", (unsigned long long)TimeNow);
  else log_w("Clock is unset, time is %llu", (unsigned long long)TimeNow);

  if (TimeNow < 1697031337) {
    NextIOTPlotterPostTimeSeconds = TimeNow + SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES;
    NextIOTPlotterHeapPostSeconds = TimeNow + (3 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
    LastWiFiCheckTime = LastPostHeapTimeSeconds = TimeNow;
  }
  ElegantOTA.loop();
  if (!ProcessingOTA) {
    log_d("Retrieving saved values from NVS");
    readNVS();

    log_i("Initializing modes");
    updateModes(true);  // Read the mode files

    LastWiFiCheckTime = getNow();
    log_i("Requesting mqtt connection to %s:%d (%s)", MQTTserver, MQTT_PORT, MQTTserverAddress);
    startMQTT();
  }
  ElegantOTA.loop();
  if (ProcessingOTA) {
    log_e("processing OTA");
    //log_w("Busy in OTA, wait...");
  }
  log_w("Previous reboot cause was %s", BootReasonText);

  ConnectNetwork();

  log_w("Last full reset was %lu minutes ago", GetUpMinutes());
  // When should we next post to IOTPlotter?

  if (NetworkGood() && ESP_RST_PANIC != lastbootreason) {
    connectToMqttCheck();
    do_delay(SleepDuration * 2);
    BlankRetainedRadarMQTT();
  }

  if (ForceInitStaticMemory) {
    // Blank out all of our RTC_NOINIT_ATTR globals
    LastRadarMessageTime = 0;
    BodySignBaseVal = 254;
    LastIOTPlotterAttemptTime = getNow();
    LastHealthCheck = 0;
    LastHealthCheckSuccess = 0;
  } else {
    if (BodySignBaseVal > 254) BodySignBaseVal = 254;
  }

// this defaults to false
#ifdef LITELED
  UseLedStrip = startLedStrip();
  if (UseLedStrip) log_i("Started LEDs");
  else {
    log_e("failed to start LEDs!!");
    delay(SleepDuration);
  }
#else
  log_i("Compiled without LEDs");
#endif

  TaskHandle_t WatchedTask = NULL;
  xTaskCreatePinnedToCore(
    BigLoop,       // Task function
    "BigLoop",     // Task name
    16 * 1024,     // Stack size in bytes (default was 8kB)
    NULL,          // Parameter
    1,             // Priority
    &WatchedTask,  // Task handle
    APP_CPU        // Core ID (0 or 1)
  );
  if (NULL == WatchedTask) {
    log_e("Failed to create new BigLoop task!");
    MustFeedWatchDog = false;
  }
  log_w("Startup Complete");
}



///////////////////////////////
inline void DoPurge() {
    make_state_topic(); // Populate state_topic from current name
  if (0 == strcmp(Where, OldName)) {
    log_i("Name has not changed, nothing to do!");
    bzero(OldName, sizeof(OldName));
    return;
  }
  log_w("Purging old MQTT retained events referencing name '%s'", OldName);
  // Wipe out records of the old name.
  bzero(MQTTreport, sizeof(MQTTreport));
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, OldName);
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);  // Purge retained
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/healthcheck", OldName);
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/bootreason", OldName);
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/memory", OldName);
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "sensor/%s/%s", OldName, BodySignString);
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "sensor/%s/%s", OldName, "presence");
  mqttClient.publish(MQTTtopic, 1, true, MQTTreport);
  feed_watchdog();
  log_w("Purged old MQTT retained events referencing name '%s'", OldName);
  bzero(OldName, sizeof(OldName));
}


//////////////////////////////////////////
// Legacy loop task
void loop() {
  time_t now;
  for (;;) {
#ifdef NEWELEGANTOTA
    ElegantOTA.loop();  // Process OTA if necessary.
    delay(1);
    now = getNow();
#else
    delay(500 * DECAYINTERVAL);
#endif
//force_interval_reboot();
#ifdef LITELED
    if (now - LastLedUpdate > DECAYINTERVAL) {
      LastLedUpdate = now;
      // It has been DECAYINTERVAL seconds since we adjusted LED brightness
      DecayBodySign();
    }
#endif
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (!ProcessingOTA && difftime(now, LastUptimePublished) > UptimePublishInterval) {
      LastUptimePublished = now;
      // Sporadically print out uptime
      snprintf(UptimeString, sizeof(UptimeString), "Uptime is %lu minutes, IP=%s.", GetUpMinutes(), MyIPAddress);
      log_d("%s", UptimeString);
      mqttClient.publish(uniqueID(), 0, false, UptimeString);
    }
  }  // End for loop
}

/////////////////////////////////////////////////////////
void BigLoop(void *arg) {
  static unsigned long loops = 0;
  log_w("BigLoop sleeping...");
  delay(7 * 1000);

#ifdef WDT_TIMEOUT
  //uint32_t coremask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1;
  //coremask = 1 << 1;  // Force mask
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << configNUM_CORES) - 1,  // Monitor all cores
    .trigger_panic = true                          // Reboot on timeout
  };
  // First must de-init the default WDT settings to clear state
  log_i("Calling esp_task_wdt_deinit() to clear Watchdog");
  if (ESP_OK != esp_task_wdt_deinit()) log_e("Watchdog failed to esp_task_wdt_deinit()");

  MustFeedWatchDog = false;
  if (ESP_OK == esp_task_wdt_init(&twdt_config) || ESP_OK == esp_task_wdt_reconfigure(&twdt_config)) {
    if (ESP_OK == esp_task_wdt_add(xTaskGetCurrentTaskHandle())) {
      WatchedTask = xTaskGetCurrentTaskHandle();
      MustFeedWatchDog = true;
      log_w("Watchdog is watching '%s' with %d wdt timeout", pcTaskGetName(WatchedTask), WDT_TIMEOUT);
    } else log_e("Failed to watch task '%s' via esp_task_wdt_add", pcTaskGetName(xTaskGetCurrentTaskHandle()));
  } else log_e("Watchdog failed to wdt_init()");
#endif  // WDT_TIMEOUT

#ifdef DOHEALTHCHECK
  if (!ProcessingOTA && NetworkGood()) {
    if (!strlen(HealthCheckPingURL)) {
      log_w("Registering with '%s'", HEALTHCHECKS_SERVER);
      RemoteHealthCallRegister();
      if (strlen(HealthCheckPingURL)) log_d("Healthchecks.io URL set to '%s'.", HealthCheckPingURL);
    }
    // Maybe we have a URL now?
    if (!strlen(HealthCheckPingURL)) log_d("No Healthcheck");
    else if (difftime(TimeNow, LastHealthCheck) > HealthCheckInterval)
      if (200 != handleDoHealthCheck()) LastHealthCheck = TimeNow - HealthCheckInterval + 120;
  }
#endif

  if (connectedMQTT) {
    snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/bootreason", Where);
    mqttClient.publish(MQTTtopic, 1, true, BootReasonText);
  }

  for (;; loops++) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
    getNow();
#ifdef WDT_TIMEOUT
    if (MustFeedWatchDog) {
      esp_task_wdt_reset();  // tickle the watchdog
      log_d("esp_task_wdt_reset() in '%s' fed watchdog", pcTaskGetName(xTaskGetCurrentTaskHandle()));
    }
#endif
    vTaskDelay(1);
    if (ProcessingOTA) {
      log_w("Processing OTA!");
      do_delay(200);
    }

    if (!ProcessingOTA) {
      if ((!LastRadarMessageTime && !TimeNow % 1200) || difftime(TimeNow, LastRadarModeSetTime) > RadarSetModeInterval) {
        handleRadarMode();
        LastRadarModeSetTime = TimeNow;
        log_d("Set Radar mode.");
      }
      vTaskDelay(1);
      if (!strlen(HealthCheckPingURL)) log_d("No Healthcheck");
      else if (difftime(TimeNow, LastHealthCheck) > HealthCheckInterval)
        if (200 != handleDoHealthCheck()) LastHealthCheck = TimeNow - HealthCheckInterval + 120;
      vTaskDelay(1);
      if (!UpdatedModes) {
        log_d("Reading modes");
        updateModes();        // Read the mode files
        UpdatedModes = true;  // just so we only call it once.
        if (strlen(OldName)) DoPurge();
        vTaskDelay(1);
      }

      // Read the radar
      DoHumanStaticPresenceLite();
      do_delay(SleepDuration);

      if (difftime(TimeNow, lastClockSet) >= SECONDS_BETWEEN_SETTING_CLOCK) {
        log_i("Setting clock");
        feed_watchdog();  // tickle the watchdog
        if (NetworkGood() && LastWebRequestSucceeded && !goodClock())
          startNTP(false);
        else timeClient.update();
        lastClockSet = TimeNow;
        vTaskDelay(1);
      }

      if (difftime(TimeNow, LastWiFiCheckTime) >= WiFiCheckInterval) {
        log_d("Checking  network");
        feed_watchdog();  // tickle the watchdog
        log_d("Cycle time %.f reached, checking WiFI...", difftime(TimeNow, LastWiFiCheckTime));
        if (NetworkGood(false)) {
          if (!connectedMQTT) connectToMqttCheck();
        }
        LastWiFiCheckTime = TimeNow;
      }
      if (difftime(TimeNow, LastPostHeapTimeSeconds) > PostHeapInterval) {
        log_d("Heap interval");
        handleReportHeapStack();  // Sends to remote
        LastPostHeapTimeSeconds = TimeNow;
      }
    }

  }  // End of stuff excluded while ProcessingOTA is true

  if (0 == loops || (0 == loops % 41)) {
    log_i("\nLoops=%d, Stack:%u, Heap:%lu, RadarStatus:%d, ip=%s",
          loops, (unsigned int)uxTaskGetStackHighWaterMark(NULL), (unsigned long)ESP.getFreeHeap(),
          RadarStatus(), getMyIP(true));
  }

  if (!ProcessingOTA) DoHumanStaticPresenceLite();
  do_delay(SleepDuration);
}


#ifdef ESP32POE
////////////////
// See https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFiGeneric.h
void EthernetEvent(WiFiEvent_t event) {
  switch (event) {

    case ARDUINO_EVENT_ETH_START:
      //case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started");
      ETH.setHostname(hostName());
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      //case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Connected to Ethernet successfully, awaiting IP");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      //case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.println(ETH.macAddress());
      Serial.print("Ethernet IP address ");
      Serial.println(ETH.localIP().toString());
      shortID(true);  // Force recalculation of NodeIDshort
      //if (ETH.fullDuplex()) log_i("Ethernet is %llu, full-duplex", (unsigned long long)ETH.linkSpeed());
      //else log_w("Ethernet is %llu, half-duplex", (unsigned long long)ETH.linkSpeed());
      if (0 == ETH.localIP().toString().compareTo(zeroip)) EthernetConnected = false;
      else {
        EthernetConnected = true;
        WantWiFi = false;
      }
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      //case SYSTEM_EVENT_ETH_DISCONNECTED:
      EthernetConnected = -2222;
      NetworkFailureCount++;
      Serial.println("Ethernet disconnected!");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      //case SYSTEM_EVENT_ETH_STOP:
      EthernetConnected = -3333;
      Serial.println("Ethernet stopped!");
      break;

    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("Got ARDUINO_EVENT_WIFI_READY");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected successfully, awaiting IP");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi AP!!");
      NetworkFailureCount++;
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("Got WiFI IP address");
      break;

    default:
      log_w("Unknown network event %d", event);
      break;
  }
}
#endif


////////////////
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Network connected OK");
}

#ifdef ESP32POE
void EthernetStart(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Ethernet started");
  ETH.setHostname(hostName());
}


////////////////
void EthernetGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  char *myIP = FindMyIP();
  log_w("ETH MAC: %s, IPv4: %s", ETH.macAddress().c_str(), myIP);
  if (ETH.fullDuplex()) log_i("Ethernet is %llus, full-duplex", (unsigned long long)ETH.linkSpeed());
  else log_i("Ethernet is %llu, half-duplex", (unsigned long long)ETH.linkSpeed());
  EthernetConnected = true;
  WantWiFi = false;
  if (strncmp(TESTNETPREFIX, myIP, strlen(TESTNETPREFIX)))
    log_d("Using existing MQTT server '%s'", MQTTserver);

#ifdef MQTT_SERVER_BACKUP
  else if (!strcmp(MQTT_SERVER, MQTTserver)) {
    strcpy(MQTTserver, MQTT_SERVER_BACKUP);
    log_w("Using backup MQTT server '%s'", MQTTserver);
  }
#endif
}
#endif  //#ifdef ESP32POE


////////////////
// call back function when IP is Lost by Station -- needs to reconnect to AP
void WiFiStationLostIP(arduino_event_id_t event) {
  Serial.println("Enter into LOST IP Event Call Back...");
}


////////////////
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  int err = 0;
  char *myIP = FindMyIP();
  log_w("WiFiEvent got IP %s", myIP);
  if (!connectedMQTT) {
    if (strncmp(TESTNETPREFIX, myIP, strlen(TESTNETPREFIX)))
      log_i("Using primary MQTT server");
#ifdef MQTT_SERVER_BACKUP
    else if (!strcmp(MQTT_SERVER, MQTTserver)) {
      strcpy(MQTTserver, MQTT_SERVER_BACKUP);
      log_w("Using backup MQTT server '%s'", MQTTserver);
    }
#endif
    IPAddress test;
    feed_watchdog();
    err = WiFi.hostByName(MQTTserver, test);

    if (1 == err) {
      log_i("DNS Resolved %s to %s successfully", MQTTserver, test.toString().c_str());
      log_w("Using MQTT server %s (%s)", MQTTserver, test.toString().c_str());
      //connectToMqttCheck();
    } else {
      log_e("Failure to DNS resolve %s, returned error code %d, using nameserver %s",
            MQTTserver, err, WiFi.dnsIP().toString());
      setDNSpublic();
    }
  }

  Serial.println("Setting timeClient");
  timeClient.update();
}


////////////////
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  log_e("Lost connection to   WiFi");
  NetworkFailureCount++;
  getMyIP(true);  // Get IP and force DHCP if necessary
}


///////////////////////////////////////
//
void StartEthernet() {
#ifdef ESP32POE
  if (0 == EthernetConnected) {
    do_delay(350);
    log_i("Starting wired Ethernet connection");
    WiFi.onEvent(EthernetEvent);  // Must call onEvent before calling begin

    // First time starting ethernet
    EthernetConnected = -1000;
    log_w("Clearing Ethernet PHY");
    pinMode(ETH_PHY_POWER, OUTPUT);
    digitalWrite(ETH_PHY_POWER, LOW);
    delay(200);  // Give ethernet some time to stabilize
    log_w("Starting up Ethernet with ETH.begin( )");
    int success = ETH.begin();
    if (success) log_w("Started Ethernet");
    else log_e("Ethernet begin failure");

  } else log_e("Ethernet already began (%d), will not begin again", EthernetConnected);
#endif
}


////////////////
boolean ConnectNetwork() {
  if (EthernetConnected > 0) return true;
  log_i("Bringing up network connection");
  StartEthernet();
  do_delay(SleepDuration);
  log_w("Ethernet MAC is %s", ETH.macAddress().c_str());
  if (strcmp("00:00:00:00:00:00", ETH.macAddress().c_str())) {
    //log_w("Connected to Ethernet, disabling WiFi");
    //WantWiFi = false;
    ETH.setHostname(hostName());
    do_delay(SleepDuration);
    log_w("ETH MAC: %s ", ETH.macAddress().c_str());

    for (int tries = 0; tries < 29 && EthernetConnected < 1; tries++) {
      Serial.print("e");
      do_delay(SleepDuration);
    }
    Serial.println(".");
    if (EthernetConnected > 0) {
      log_i("Ethernet UP");
      WantWiFi = false;
      return (true);
    }
  } else {
    log_w("No Ethernet!");
  }

  boolean ready = NetworkGood(false);
  if (!ready) {
    Serial.print("Waiting on network...");
    for (int i = 0; i < 20 && !ready; i++) {
      do_delay(SleepDuration);
      ready = NetworkGood(false);
    }
  }
  if (ready) log_i("Network up.");
  else log_w("Network is still not up.");
  return (ready);
}


////////////////
// getNow
time_t getNow() {
  if (DateTime.isTimeValid())
    return (TimeNow = DateTime.getTime());

  if (NetworkGood()) {
    timeClient.update();
    if (timeClient.isTimeSet()) return TimeNow = timeClient.getEpochTime();
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec > 1697031337) return (TimeNow = tv.tv_sec);

  //#ifndef USE_NTP
  //  return (TimeNow = tv.tv_sec);
  //#endif

  log_w("fallback to esp_timer");
  return (TimeNow = esp_timer_get_time() / 1000000);
}


////////////////
inline boolean goodClock() {
  if (timeClient.isTimeSet()) {
    WaitingForClock = false;
    return true;
  } else return false;
}


////////////////
// Somewhat generic HTTPS post, content_type is optional
String &doPOST(const char *url, char *PostDataBuffer, const char *apikey, const char *contenttype = "application/json") {
  //static String payload;  // Static so we can return a reference
  static String payload = "Empty";
  static char result[32] = "failure HTTPS";
  int ecode = 0;
  int httpResponseCode = 0;
  log_i("Performing %d byte POST to %s", strlen(PostDataBuffer), url);
  LastWebRequestSucceeded = false;  // Set the global
  client->setInsecure();
  feed_watchdog();
  {
    // Force scoping so HTTPclient is destroyed before WifiClientSecure client
    HTTPClient http;
    //char ua[URLBUFSIZE] = "Mozilla/5.0 (42) Firefox/78.0";
    http.setUserAgent("Mozilla/5.0 (42) Firefox/78.0");

    log_d("http.begin '%s'", url);
    int err = http.begin(*client, url);
    if (0 == err) {
      log_w("HTTPS failure in begin() to '%s'", url);
      LastWebRequestSucceeded = false;
      feed_watchdog();
      bzero(LastWebError, sizeof(LastWebError));
      snprintf(LastWebError, sizeof(LastWebError) - 2, "HTTPS failure in begin() to '%s'", url);
      publishError(LastWebError);
      payload = String("failure in begin");

      return (payload);
    } else {
      log_d("POSTing");
      // http.setTimeout(HTTP_TIMEOUT_DATA);              // Data timeout
      // http.setConnectTimeout(HTTP_HANDSHAKE_TIMEOUT);  // Setup timeout
      // log_i("Set main timeout to %d seconds, and handshake timeout to %d seconds", HTTP_TIMEOUT_DATA, HTTP_HANDSHAKE_TIMEOUT);
      //http.addHeader("User-Agent", VERSION);
      http.addHeader("Access-Control-Request-Headers", "*");
      if (contenttype && strlen(contenttype)) http.addHeader("Content-Type", contenttype);
      if (apikey && strlen(apikey)) {
        http.addHeader("api-key", apikey);
        http.addHeader("X-Api-Key", apikey);
      }
      feed_watchdog();
      log_i("Sending '%s' to %s", PostDataBuffer, url);
      httpResponseCode = http.POST((unsigned char *)PostDataBuffer, strlen(PostDataBuffer));
      feed_watchdog();
      log_i("Posted, got response code %d", httpResponseCode);
      LastWebRequestCode = httpResponseCode;  // Save to global

      if (httpResponseCode < 1) {
        bzero(LastWebError, sizeof(LastWebError) - 2);
        snprintf(LastWebError, sizeof(LastWebError) - 2, "POST to %s failure with %d", url, httpResponseCode);
        publishError(LastWebError);
        payload = String(httpResponseCode);
      } else {
        log_d("HTTP Response code: %d", httpResponseCode);
        payload = http.getString();
        if (199 < httpResponseCode && httpResponseCode < 300) {
          LastWebRequestSucceeded = true;
          log_i("Success, got %s", payload.c_str());
          sprintf(result, "Response code %d", httpResponseCode);
          strncpy(LastWebError, result, sizeof(LastWebError) - 2);
          log_i("Last web request returned %d and %s", httpResponseCode, LastWebError);
          ecode = httpResponseCode;
          LastWebRequestCode = ecode;
        } else {
          log_w("HTTPS Error code: %d", httpResponseCode);
          bzero(LastWebError, sizeof(LastWebError));
          strncpy(LastWebError, payload.c_str(), sizeof(LastWebError) - 2);
          log_e("Error %d, payload '%s'", httpResponseCode, LastWebError);

          sprintf(result, "ERRNO=%d", httpResponseCode);
          ecode = httpResponseCode;
          if (ecode > 299) ecode = -1 * httpResponseCode;  // Force it negative
          LastWebRequestCode = ecode;                      // Copy to global state
        }
      }
      // Free resources
      http.end();
    }
    feed_watchdog();
    log_d("Done with HTTP");
  }
  bzero(LastWebError, sizeof(LastWebError));
  if (client->lastError(LastWebError, BUFFER_SIZE - 1) < 0) {

    client->lastError(LastWebError, sizeof(LastWebError) - 2);
    log_w("Last https error %d was %s", httpResponseCode, LastWebError);
  }
  log_d("Will return %d bytes", strlen(payload.c_str()));
  log_i("Successfully finished HTTP post to '%s'", url);
  return (payload);
}


////////////////
boolean startNTP(boolean loopuntilgood) {
  if (!NetworkGood()) {
    log_w("No network connection to do NTP over");
    return false;
  }
  log_d("Checking time");
  if (!timeClient.isTimeSet()) {
    log_i("Starting timeClient");
    timeClient.begin();
    timeClient.update();
  } else WaitingForClock = false;
  do_delay(SleepDuration / 2);
  log_d("Rechecking time");
  if (timeClient.isTimeSet()) WaitingForClock = false;
  else {
    WaitingForClock = true;
    log_w("failure to start timeClient");
    setenv("TZ", TZ, 1);
    tzset();
    do_delay(SleepDuration / 2);
    return false;
  }
  // TimeClient successful
  log_i("Setting system clock with TZ=%s", TZ);
  struct timeval tv;
  tv.tv_sec = timeClient.getEpochTime();
  settimeofday(&tv, NULL);
  setenv("TZ", TZ, 1);
  tzset();
  log_i("Started timeClient");
  return true;
}




////////////////
void setupwebserver(const char *user, const char *pass) {
  log_i("Creating HTTP server with authentication requiring %s", user);
  // Route for root / web page

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", index_html, processor);
        })
    .setAuthentication(user, pass);

  server.on("/wantota", HTTP_GET, [](AsyncWebServerRequest *request) {
          ProcessingOTA = true;
          request->redirect("/update");
        })
    .setAuthentication(user, pass);

  // Speed up all the timers to make memory leaks happen faster!
  server.on("/runfast", HTTP_GET, [](AsyncWebServerRequest *request) {
          InfluxMQTTPublishInterval = SECONDS_BETWEEN_INFLUXMQTT_PUBLISHES / 3;
          MQTTPublishInterval = SECONDS_BETWEEN_MQTT_PUBLISHES / 3;
          RadarSetModeInterval = RADARSETMODEINTERVAL / 2;
          SleepDuration = SLEEPDURATION / 2;
          RadarResetInterval = RADARRESETINTERVAL / 2;
          PostHeapInterval = POSTHEAPINTERVAL / 2;
          WiFiCheckInterval = WIFICHECKINTERVAL / 3;
          ForcedRadarIotPlotterInterval = FORCEDRADARIOTPLOTTERINTERVAL / 3;
          ForcedRadarNoMessagePublishInterval = FORCEDRADARNOMESSAGEPUBLISHINTERVAL / 4;
          HealthCheckInterval = HEALTHCHECKINTERVAL / 4;  // Really often
          UptimePublishInterval = UPTIMEPUBLISHINTERVAL / 2;
          log_w("/runfast decreased intervals!");
          request->redirect("/stats");
        })
    .setAuthentication(user, pass);

  // Force restart via authenticated web GET request
  server.on("/forcerestart", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->redirect("/");
          // Replaced "Where" with "Room"
          snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
          mqttClient.setWill(MQTTtopic, 1, true, "black");  // Mark sensor as off/vacant on drop
          mqttClient.publish(uniqueID(), 1, true, "forcerestart");
          mqttClient.disconnect(false);
#ifdef LITELED
          if (UseLedStrip) {
            strip.clear(true);
            strip.brightness(0, true);
          }
#endif
          ESP.restart();
        })
    .setAuthentication(user, pass);

#ifdef DOHEALTHCHECK
  server.on("/rhc", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", health_html, processor);
        })
    .setAuthentication(user, pass);

  //  Ping the remotehealthcheck server
  server.on("/rhcping", HTTP_GET, [](AsyncWebServerRequest *request) {
          log_i("Called /rhcping");
          snprintf(UptimeString, sizeof(UptimeString), "Uptime is %lu minutes.", GetUpMinutes());
          RemoteHealthCall(1, UptimeString, "Via Web UI");
          request->send_P(200, "text/html", health_html, processor);
        })
    .setAuthentication(user, pass);

  // Re-register to remotehealthcheck server
  server.on("/rhcregister", HTTP_GET, [](AsyncWebServerRequest *request) {
          log_w("Called /rhcregister");
          RemoteHealthCallRegister(true);
          request->send_P(200, "text/html", health_html, processor);
        })
    .setAuthentication(user, pass);
#endif

  server.on("/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
          connectToMqtt();
          request->send_P(200, "text/html", mqtt_html, processor);
        })
    .setAuthentication(user, pass);

#ifdef USE_GRAPHER
  server.on("/mqttgraph", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", mqttgraph_html, processor);
        })
    .setAuthentication(user, pass);
#endif


  // mqttserverform_html
  server.on("/mqttserver", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, TextHtml, mqttserverform_html, processor);
        })
    .setAuthentication(user, pass);

  // Handle their form answer
  server.on("/mqttserver", HTTP_POST, [](AsyncWebServerRequest *request) {
          //char host[MIDSIZESTRING];
          int params = request->params();
          log_d("%d params sent in", params);
          for (int i = 0; i < params; i++) {
            AsyncWebParameter *p = request->getParam(i);
            if (p->isPost()) {
              if (0 == strcmp("mqttserver", p->name().c_str())) {
                // We have a Server to process
                const char *recd = p->value().c_str();
                if (strlen(recd) < 6) {
                  char errmessage[MSGBUFFER_SIZE + 32];
                  bzero(errmessage, sizeof(errmessage));
                  snprintf(errmessage, sizeof(errmessage) - 1, "Too short name '%s'", recd);
                  request->send_P(200, TextPlain, errmessage);
                  log_e("%s", errmessage);
                } else {
                  if (SavedNVS.begin(TAG, PREF_RW)) {
                    log_d("Updating saved MQTTserver in NVS to %s", MQTTserver);

                    if (!strcmp("delete", recd)) {
                      if (SavedNVS.remove(nvs_MQTTServer)) {
                        log_i("Deleted saved MQTT server");
                        request->send_P(200, "text/html", "<HTML>Updated! [<A HREF=/mqttserver>Go Back</A></HTML>");
                      } else {
                        log_e("Failed to removed NVS entry for '%s'", nvs_MQTTServer);
                        request->send_P(200, "text/html", "<HTML>Failed to update! [<A HREF=/mqttserver>Go Back</A></HTML>");
                      }
                    } else {
                      SavedNVS.putString(nvs_MQTTServer, recd);
                      SavedNVS.end();
                      bzero(MQTTserver, sizeof(MQTTserver));
                      strncpy(MQTTserver, recd, sizeof(MQTTserver) - 1);
                      request->send_P(200, "text/html", "<HTML>Updated! [<A HREF=/mqttserver>Go Back</A></HTML>");
                    }
                  } else {
                    log_e("Could not open NVS to save MQTTserver");
                    request->send_P(200, TextHtml, "<HTML>NVS Fail! [<A HREF=/MQTTserver>Go Back</A></HTML>");
                  }
                }
              }
            }
          }
        })
    .setAuthentication(user, pass);


  // mqttws31_js
#ifdef USE_GRAPHER
  server.on("/mqttws31.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/javascript", mqttws31_js);
  });
#endif

  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", stats_html, processor);
        })
    .setAuthentication(user, pass);

  server.on("/iotplotter", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", iotplotter_html, processor);
        })
    .setAuthentication(user, pass);


  server.on("/watch", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", watch_html, processor);
        })
    .setAuthentication(user, pass);

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send_P(200, "text/html", config_html, processor);
        })
    .setAuthentication(user, pass);

  server.on(
          "/configure", HTTP_POST, [](AsyncWebServerRequest *request) {
            boolean wantdisable = false;
            int params = request->params();
            log_i("%d params sent to /configure", params);
            boolean nvsok = (SavedNVS.begin(TAG, PREF_RW));
            for (int i = 0; i < params; i++) {
              AsyncWebParameter *p = request->getParam(i);
              if (p->isPost()) {
                log_i("%s: %s", p->name().c_str(), p->value().c_str());
                if (!strcmp("name", p->name().c_str()) && p->value().length()) {
                  strcpy(OldName, Where);
                  bzero(Where, sizeof(Where));
                  strncpy(Where, p->value().c_str(), sizeof(Where) - 1);
                    make_state_topic(); // Populate state_topic from current name
                  UpdatedModes = false;
                  if (nvsok && strlen(Where) && strcmp(Where, TAG))
                    SavedNVS.putString(nvs_whereword, p->value());
                } else if (!strcmp(nvs_Floor, p->name().c_str()) && p->value().length()) {
                  bzero(Floor, sizeof(Floor));
                  strncpy(Floor, p->value().c_str(), sizeof(Floor) - 1);
                  UpdatedModes = false;
                  if (nvsok && strlen(Floor))
                    SavedNVS.putString(nvs_Floor, p->value());
                } else if (0 == strcmp(nvs_DisableSensor, p->name().c_str()) && p->value().length()) {
                  log_w("Removing old '%s' value, setting it to '%s'", DisableSensor, p->value().c_str());
                  bzero(DisableSensor, sizeof(DisableSensor));
                  UpdatedModes = false;
                  if (0 == strcmp("enable", p->value().c_str())) {
                    log_w("User set disablesensor to 'enable'!");
                    wantdisable = false;
                  } else {
                    strncpy(DisableSensor, p->value().c_str(), sizeof(DisableSensor) - 1);
                    wantdisable = true;
                  }
                  if (nvsok) {
                    // Save the Room
                    if (strlen(DisableSensor)) {
                      SavedNVS.putString(nvs_DisableSensor, p->value());
                      DisabledSensor = true;
                      UpdatedModes = false;
                      log_w("Sensor is disabled with string '%s'", DisableSensor);
                    } else {
                      log_w("Sensor is enabled.");
                      if (SavedNVS.isKey(nvs_DisableSensor)) SavedNVS.remove(nvs_DisableSensor);
                      DisabledSensor = false;
                      UpdatedModes = false;
                    }
                  }
                } else if (!strcmp(nvs_Room, p->name().c_str()) && p->value().length()) {
                  bzero(MQTTtopic, sizeof(MQTTtopic));
                  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, OldName);
                  mqttClient.publish(MQTTtopic, 1, true, "");  // Purge retained
                  bzero(Room, sizeof(Room));
                  strncpy(Room, p->value().c_str(), sizeof(Room) - 1);
                  build_nowhere();  // Populate nowhere
                  UpdatedModes = false;
                  if (nvsok && strlen(Room))
                    SavedNVS.putString(nvs_Room, p->value());
                }
              }
            }
            if (nvsok && !wantdisable) {
              log_w("Sensor is enabled.");
              DisabledSensor = false;
              UpdatedModes = false;  // Indicates we need an update
            }
            if (nvsok) SavedNVS.end();
            else log_w("Failed to write to NVS!");
            request->send_P(200, "text/html", index_html, processor);
          })
    .setAuthentication(user, pass);

#ifdef NEWELEGANTOTA
  // Start ElegantOTA
  ElegantOTA.begin(&server);
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  ElegantOTA.setAuth(user, pass);
  log_i("Configured ElegantOTA to require user '%s'", user);
#endif

  // Start server
  server.begin();
}



////////////////////
const char *uniqueID(boolean force) {
  static boolean populated = false;
  if (populated && !force && strlen(NodeID) > 5) return (NodeID);

  snprintf(NodeID, sizeof(NodeID) - 1, "%s-%s", TAG, shortID(true));
  log_i("Set NodeID to '%s'", NodeID);

  if (strlen(NodeID) && !strstr(NodeID, "000000")) {
    populated = true;
    return (NodeID);
  }

#ifdef USEBASEMAC
  uint8_t mac[7];
  bzero(mac, 7);
  log_d("Falling back to esp_read_mac");

#ifdef ESP_MAC_ETH
  esp_read_mac(mac, ESP_MAC_ETH);
#endif

  if (!mac[0] && !mac[1] && !mac[2]) {
#ifdef ESP_MAC_WIFI_STA
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
  }

  snprintf(NodeID, sizeof(NodeID) - 1, "%s%s-%02X%02X%02X%02X%02X%02X", DeviceType, TAG, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (strlen(NodeID) && !strstr(NodeID, "000000")) {
    return (NodeID);
  }
  snprintf(NodeID, sizeof(NodeID) - 1, "%s-%s", TAG, ETH.macAddress().c_str());
  RemoveCharacter(':', NodeID);
  if (!strstr(NodeID, "000000")) {
    populated = true;
    return (NodeID);
  } else bzero(NodeID, sizeof(NodeID));  // Nulls are no good to us.
#endif
  if (!strlen(NodeID)) {
    log_i("Fell back to efuseMac");
    snprintf(NodeID, sizeof(NodeID) - 1, "%012llx", ESP.getEfuseMac());
  }
  return (NodeID);
}


////////////////
const char *hostName() {
  static char id[BUFFER_SIZE];

  uint8_t mac[7];
  //esp_read_mac(mac, ESP_MAC_WIFI_STA);
  WiFi.macAddress(mac);
  //if(! (mac[0] || mac[1] || mac[2] || mac[3] || mac[4])) ETH.getMac(mac);
  if (!(mac[0] || mac[2] || mac[3] || mac[4])) {
    esp_efuse_mac_get_default(mac);
    log_w("Using efuse_mac %02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  snprintf(id, BUFFER_SIZE - 1, "%.31s-%02X%02X%02X%02X%02X%02X", Where, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return (id);
}


////////////////////
//
const char *shortID(boolean force) {
  static boolean populated = false;
  if (populated && !force) return (NodeIDshort);
  log_d("Repopulating shortID(%d)", force);
  bzero(NodeIDshort, sizeof(NodeIDshort));

  strncpy(NodeIDshort, ETH.macAddress().c_str(), sizeof(NodeIDshort) - 1);
  log_i("Got ETH MAC %s", NodeIDshort);
  if (!strncmp("00:00:00:00:00:00", NodeIDshort, 8)) {
    // Bogus MAC, try  WiFi.macAddress
    strncpy(NodeIDshort, WiFi.macAddress().c_str(), sizeof(NodeIDshort) - 1);
    log_i("Fell back to WiFi MAC %s", NodeIDshort);
    if (!strncmp("00:00:00:00:00:00", NodeIDshort, 8)) {
      // Still no good
      int64_t chipid = ESP.getEfuseMac();
      // Format the MAC address as colon-separated hex values
      // The MAC is a 6-byte value; the high 2 bytes and low 4 bytes are accessed separately.
      snprintf(NodeIDshort, sizeof(NodeIDshort) - 1, "%02X:%02X:%02X:%02X:%02X:%02X",
               (uint8_t)(chipid >> 40), (uint8_t)(chipid >> 32), (uint8_t)(chipid >> 24),
               (uint8_t)(chipid >> 16), (uint8_t)(chipid >> 8), (uint8_t)(chipid >> 0));
      log_i("Fell back to efuse MAC %s", NodeIDshort);
    }
  }
  RemoveCharacter(':', NodeIDshort);
  populated = true;
  log_d("NodeIDshort now '%s'", NodeIDshort);
  return (NodeIDshort);
}



////////////////
void logram(void) {
  log_i("Total heap: %d, Free heap: %d", ESP.getHeapSize(), ESP.getFreeHeap());
  log_i("Biggest heap fragment: %d", ESP.getMaxAllocHeap());
  //log_i("Total PSRAM: %d", ESP.getPsramSize());
  //log_i("Free PSRAM: %d", ESP.getFreePsram());
  //log_i("Initial task stack size %d bytes", getArduinoLoopTaskStackSize());
  log_w("Free Stack: %u bytes", (unsigned int)uxTaskGetStackHighWaterMark(NULL));
  //log_i("Total flash storage %s", humanReadableSize((size_t)ESP.getFlashChipSize()));
}

esp_reset_reason_t logbootreason(void) {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:
      strcpy(BootReasonText, "Unknown Reboot");
      log_w("Reset reason can not be determined");
      ForceInitStaticMemory = 1;
      break;

    case ESP_RST_POWERON:
      strcpy(BootReasonText, "Power-on boot");
      log_w("Reset due to power-on event");
      ForceInitStaticMemory = 1;
      break;

    case ESP_RST_EXT:
      log_w("Reset by external pin");  //  (not applicable for ESP32)
      break;

    case ESP_RST_SW:
      strcpy(BootReasonText, "Soft Reboot");
      log_w("Software reset via esp_restart");
      break;

    case ESP_RST_PANIC:
      strcpy(BootReasonText, "Panic Reboot");
      log_w("Software reset due to exception/panic");
      break;

    case ESP_RST_INT_WDT:
      strcpy(BootReasonText, "IRQ Watchdog");
      log_w("Reset (software or hardware) due to interrupt watchdog");
      break;

    case ESP_RST_TASK_WDT:
      strcpy(BootReasonText, "Task Watchdog");
      log_w("Reset due to task watchdog");
      break;

    case ESP_RST_WDT:
      strcpy(BootReasonText, "Other Watchdog");
      log_w("Reset due to other watchdogs");
      break;

    case ESP_RST_DEEPSLEEP:
      strcpy(BootReasonText, "Deep Reboot");
      log_w("Reset after exiting deep sleep mode");
      break;

    case ESP_RST_BROWNOUT:
      strcpy(BootReasonText, "Brownout Boot");
      log_w("Brownout reset (software or hardware)");
      break;

    case ESP_RST_SDIO:
      strcpy(BootReasonText, "Reset over SDIO");
      log_w("Reset over SDIO");
      break;

    default:
      snprintf(BootReasonText, sizeof(BootReasonText) - 1, "Unknown Boot %X", reason);
      log_w("Unknown Boot %X", reason);
      ForceInitStaticMemory = 1;
      break;
  }
  log_w("Processed reboot reason %s", BootReasonText);
  return (reason);
}

// Remove character in place
char *RemoveCharacter(char unwanted, char *str) {
  char *read_ptr = str;
  char *write_ptr = str;

  while (*read_ptr != '\0') {
    if (*read_ptr == unwanted) read_ptr++;
    else {
      // Copy non-space characters or spaces inside quoted strings
      *write_ptr = *read_ptr;
      read_ptr++;
      write_ptr++;
    }
  }
  *write_ptr = '\0';
  return (str);
}

////////////////
//   Uses the output of difftime
char *humanReadableLongDuration(long timestamp, bool noseconds) {
  static char duration[BUFFER_SIZE];
  if (0 == timestamp || (noseconds && (60 > abs(timestamp)))) {
    strcpy(duration, "now");
    return duration;
  }

  if (timestamp < 0) timestamp = abs(timestamp);
  long days = timestamp / 86400;
  unsigned long ts = (unsigned long)timestamp - (86400 * days);

  int hours = (ts % 86400) / 3600;
  int minutes = (ts % 3600) / 60;
  int seconds = ts % 60;

  // If on an exact minute, do not send the seconds
  if (0 == seconds) noseconds = true;

  if (days) {
    snprintf(duration, sizeof(duration) - 1, "%lddays, %dh%dm", days, hours, minutes);
    return (duration);
  }
  if (hours) {
    snprintf(duration, sizeof(duration) - 1, "%dh%dm", hours, minutes);
    return (duration);
  }

  if (minutes || noseconds) {
    if (noseconds) snprintf(duration, sizeof(duration) - 1, "%dmin", minutes);
    else snprintf(duration, sizeof(duration) - 1, "00:%2.2d:%2.2d", minutes, seconds);
    return (duration);
  }
  snprintf(duration, sizeof(duration) - 1, "%ldsecs", timestamp);
  return (duration);
}


////////////////
String processor(const String &var) {
  String result = String("<") + var + String(">");
  //log_d("Entering processor(%s)", var.c_str());

  if (var[0] == 'B') {
    if (var == "BODYSIGNPEAK") return (String(BodySignPeakVal));
    if (var == "BODYSIGNBASE") return (String(BodySignBaseVal));
    if (var == "BODYSIGN") return (String(BodySignVal));
    if (var == "BOOTREASON") return String(BootReasonText);
    if (var == "BOOTTIME") return String(DateTime.getBootTime());
    if (var == "BIGJSONBUFFER") {
#ifdef DOHEALTHCHECK
      return String(BigJSONbuffer);
#else
      return String("N/A");
#endif
    }
    if (var == "BUILD") {
      char ident[URLBUFSIZE];
      snprintf(ident, URLBUFSIZE - 1, "%s %s %s", __DATE__, __TIME__, __FILE__);
      ident[URLBUFSIZE - 1] = (char)0x00;
      return String(ident);
    }
    if (var == "BRIGHTNESS") {
#ifdef LITELED
      return String(StripBrightness);
#else
      return String("Compiled w/o LiteLED");
#endif
    }
    return result;
  }  // End of 'B'

  if (var[0] == 'D') {
    if (var == "DISABLED") {
      if (DisabledSensor) return String("Sending is disabled");
      return String("");
    }
    if (var == "DATETIME") return humanReadableTime(TimeNow);
    if (var == "DATETIMERTC") return humanReadableTime(TimeNow);
    if (var == "DISABLE") return String(DisableSensor);
    return result;
  }  // End of 'D'

  if (var[0] == 'F') {
    if (var == "FLOOR") return String(Floor);
    if (var == "FLOORNVS") return String(nvs_Floor);

    if (var == "FREEHEAP") return humanReadableSize((size_t)ESP.getFreeHeap());
    if (var == "FRAGHEAP") return humanReadableSize((size_t)ESP.getMaxAllocHeap());
    if (var == "FREESTACK") return humanReadableSize((size_t)uxTaskGetStackHighWaterMark(NULL));

    if (var == "FEEDWATCHDOG") {
#ifndef WDT_TIMEOUT
      return StringNever;
#else
      return humanReadableLongDuration(WDT_TIMEOUT, false);
#endif
    }

    if (var == "FEEDINGWATCHDOG") {
#ifndef WDT_TIMEOUT
      return String("Watchdog disabled");
#else
      if (MustFeedWatchDog) return String("Feeding watchdog");
      return String("No need to feed watchdog");
#endif
    }
    return result;
  }  // End of 'F'

  if (var[0] == 'H') {
    if (var == "HTTPERROR") return String(LastWebError);

#ifdef DOHEALTHCHECK
    if (var == "HCPROJECTURL") return String(HealthCheckProjectURL);

    if (var == "HEALTHCHECKURL") {
      if (strlen(HealthCheckPingURL) > 5) return String(HealthCheckPingURL);
      return StringUndef;
    }
    if (var == "HEALTHCHECKPAUSEURL") {
      if (strlen(HealthCheckPauseURL) > 5) return String(HealthCheckPauseURL);
      return StringUndef;
    }
    if (var == "HEALTHCHECKRESUMEURL") {
      if (strlen(HealthCheckResumeURL) > 5) return String(HealthCheckResumeURL);
      return StringUndef;
    }
    if (var == "HEALTHCHECKUPDATEURL") {
      if (strlen(HealthCheckUpdateURL) > 5) return String(HealthCheckUpdateURL);
      return StringUndef;
    }
    if (var == "HEALTHLASTCHECKSUCCESS") {
      if (0 >= LastHealthCheckSuccess) return StringNever;
      return humanReadableTime(LastHealthCheckSuccess);
    }
    if (var == "HEALTHLASTCHECK") {
      if (0 >= LastHealthCheck) return StringNever;
      return humanReadableTime(LastHealthCheck);
    }
    if (var == "HEALTHLASTCHECKRESULT") return String(LastHealthCheckResult);
    if (var == "HEALTHLASTCHECKCODE") return String(LastHealthCheckCode);
#else
    if (var == "HEALTHLASTCHECKRESULT" || var == "HEALTHLASTCHECKCODE" || var == "HEALTHLASTCHECK" || var == "HEALTHLASTCHECKSUCCESS" || var == "HEALTHCHECKURL") return String("No HC code compiled");
#endif
    return result;
  }  // End of 'H'


  if (var[0] == 'I') {
    if (var == "IOTPLOTTERSENT") return String(IotPlotterDataBuffer);
    if (var == "IOTPLOTTERANSWER") return String(IotPlotterResponseString);
    if (var == "IOTPLOTTERDATA") return String(IotPlotterDataBuffer);
    return result;
  }  // End of 'I'

  if (var[0] == 'L') {
    if (var == "LASTFLOORPLANVAL") return String(LastFloorplanBodySignVal);
    if (var == "LASTFLOORPLANTIME") {
      if (!LastBodySignFloorplanTimeSeconds) return StringNever;
      else return humanReadableTime(LastBodySignFloorplanTimeSeconds);
    }
    // String representing the last time we saw a message arrive from the Radar
    if (var == "LASTRADARTIME" || var == "RADARREADTIME") {
      if (0 >= LastRadarMessageTime) return StringNever;
      else return humanReadableTime(LastRadarMessageTime);
    }
    if (var == "LASTBODYSIGNMQTTTIME") {
      if (1 > LastBodySignMQTTTimeSeconds) return StringNever;
      return humanReadableTime(LastBodySignMQTTTimeSeconds);
    }

    if (var == "LASTDETAILPOSTTIME") {
      if (0 >= LastDetailPostTimeSeconds) return StringNever;
      return humanReadableTime(LastDetailPostTimeSeconds);
    }

    if (var == "LASTRADARMSG") return (String(humanReadableLongDuration((long)(TimeNow - LastRadarMessageHandledTime), false)));
    if (var == "LASTBODYSIGNPOSTVAL") return (String(LastPostedBodySignVal));
    if (var == "LASTBODYSIGNPOST") {
      if (0 >= LastBodySignPostTimeSeconds) return StringNever;
      return humanReadableTime(LastBodySignPostTimeSeconds);
    }
    if (var == "LASTSTATUSPOSTSUCCESS") {
      if (0 >= LastStatusPostSuccessTimeSeconds) return StringNever;
      return humanReadableTime(LastStatusPostSuccessTimeSeconds);
    }
    if (var == "LASTBODYSIGNPOSTSUCCESS") {
      if (0 >= LastBodySignPostSuccessTimeSeconds) return StringNever;
      return humanReadableTime(LastBodySignPostSuccessTimeSeconds);
    }
    if (var == "LASTDETAILPOSTSUCCESS") {
      if (0 >= LastDetailPostSuccessTimeSeconds) return StringNever;
      return humanReadableTime(LastDetailPostSuccessTimeSeconds);
    }

    if (var == "LASTFEDWATCHDOG") {
#ifndef WDT_TIMEOUT
      return String(NotNeededString);
#endif
      if (0 >= LastFedWatchdog) return StringNever;
      return humanReadableTime(LastFedWatchdog);
    }
    return result;
  }  // End of 'L'

  if (var[0] == 'M') {
    if (var == "MQTTPORT") return String(MQTT_PORT);
    if (var == "MQTTPORTWS") return String(MQTT_PORT_WS);
    if (var == "MQTTSERVER") return String(MQTTserver);
    if (var == "MQTTSERVERADDRESS") return String(MQTTserverAddress);

    if (var == "MQTTTOPIC") return String(Topic_Influx_Occupancy);
    if (var == "MQTTREASON") return String(MQTTReasonText);

    if (var == "MQTTCONNECTED") {
      if (connectedMQTT) return String("Connected to MQTT");
      return String("Not connected to MQTT");
    }
    return result;
  }  // End of 'M'

  if (var[0] == 'N') {
    if (var == "NVSDISABLE") return String(nvs_DisableSensor);
    if (var == "NOWHERE") return String(NoWheres.c_str());
    if (var == "NETWORKFAILURECOUNT") return (String(NetworkFailureCount));
    if (var == "NTPSERVER") return String(NTPSERVERNAME);            //return String(DateTime.getServer());
    if (var == "NTPTIME") return String(timeClient.getEpochTime());  //return String(DateTime.getTime());
    if (var == "NTPSTATUS") {
      if (timeClient.isTimeSet()) return String("SNTP good");
      else return String("SNTP Invalid");
    }
    return result;
  }  // End of 'N'

  if (var[0] == 'P') {
    if (var == "POSTDATABUFFER") return String(PostDataBuffer);
    if (var == "PLOTTERERROR") return String(LastPlotterError);
    if (var == "PLOTTERCODE") return String(LastPlotterRequestCode);
    return result;
  }


  if (var[0] == 'R') {
    if (var == "RADARMESSAGE") return String(RadarMessage);
    if (var == "RADARMESSAGECOUNT") return String(RadarMessageCount);
    if (var == "ROOM") return String(Room);
    if (var == "ROOMNVS") return String(nvs_Room);
    return result;
  }  // End of 'R'


  if (var[0] == 'S') {
    if (var == "SIZEFLASH") return humanReadableSize((size_t)ESP.getFlashChipSize());
    if (var == "SAVEDWEBERROR") {
      if (false == SavedNVS.begin(TAG, PREF_RO)) {
        return String("NVS open failure");
      }
      char buffer[ERRBUFSIZE] = "NoSavedWebError";
      SavedNVS.getString(nvs_LastWebError, buffer, (size_t)ERRBUFSIZE - 1);
      SavedNVS.end();
      return String(buffer);
    }
    return result;
  }  // End of 'S'


  if (var[0] == 'T') {
    if (var == "TOTALHEAP") return humanReadableSize((size_t)ESP.getHeapSize());
    if (var == "TOTALSTACK") return humanReadableSize((size_t)getArduinoLoopTaskStackSize());
    return result;
  }  // End of 'T'

  if (var[0] == 'U') {
    if (var == "UNAME") return String(uniqueID());
    if (var == "UPMINUTES") return String(GetUpMinutes());
    if (var == "UPTIME") {
      snprintf(UptimeString, sizeof(UptimeString), "Uptime is %lu minutes.", GetUpMinutes());
      return String(UptimeString);
    }

    if (var == "USB_MANUFACTURER") {
#ifdef USB_MANUFACTURER
      return String(USB_MANUFACTURER);
#else
      return String("Unknown ESP32");
#endif
    }

#ifdef USB_PRODUCT
    if (var == "USB_PRODUCT") return String(USB_PRODUCT);
#ifdef ESP32POE
    if (var == "USB_PRODUCT") return String("ESP32 POE");
#endif
#ifdef XIAO_ESP32C3
    if (var == "USB_PRODUCT") return String("XIAO ESP32C3");
#endif
#endif
    return result;
  }  // End of 'U'


  if (var == "VERSION") return String(VERSION);

  if (var[0] == 'W') {
    if (var == "WHERE") return String(Where);
    if (var == "WEBCODE") return String(LastWebRequestCode);
    if (var == "WEBERROR") {
      if (LastWebRequestSucceeded) return String("Last Web request was Successful");
      return String("Last Web request failure");
    }
    if (var == "WATCHEDTASK") {
      if (WatchedTask) return String(pcTaskGetName(WatchedTask));
      return String("none");
    }
    return result;
  }
  return result;
}

////////////////////
inline int handleDoHealthCheck() {
  int retcode = 0;
#ifdef DOHEALTHCHECK
  log_d("Time to run a Healthcheck");
  char *myip = FindMyIP();
  if (LastHealthCheck) {
    //  Only true on second and subsequent runs after a reboot
    bzero(PostDataBuffer, sizeof(PostDataBuffer));
    snprintf(PostDataBuffer, sizeof(PostDataBuffer) - 1, "Radar message #%ld at %s was '%s', uptime is %lu minutes, Disabled='%s'\n,%s, IOTplotter returned %s",
             RadarMessageCount, humanReadableTimeArray(LastRadarMessageTime), RadarMessage, GetUpMinutes(), DisableSensor, IotPlotterDataBuffer, IotPlotterResponseString);
    retcode = RemoteHealthCall(1, PostDataBuffer, FindMyIP());
  } else {
    snprintf(LastWebError, sizeof(LastWebError) - 1, "Post-Reboot IP %s", myip);
    retcode = RemoteHealthCall(1, BootReasonText, myip);
  }
  log_d("Local IP address is %s, ETH=%d", myip, EthernetConnected);

  if (retcode < 200 || retcode > 304) {
    log_e("Did HealthCheck, returned %d", retcode);
    do_delay(SleepDuration * 3);
  } else log_d("Did HealthCheck, returned %d", retcode);
#else
  log_i("Compiled Without HealthCheck code");
#endif
  return (retcode);
}


void ConfigureHTTPClient(HTTPClient &http, const char *agent) {
  http.setUserAgent(agent);
  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT);
  http.setReuse(false);
  http.setTimeout(HTTP_TIMEOUT_DATA);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  LastWebRequestSucceeded = false;
  log_d("Set HTTP timeouts to %d for connect, %d for data.", HTTP_CONNECT_TIMEOUT, HTTP_TIMEOUT_DATA);
}


/////////////////////////////////////////
// Return current time as a float
float getNowFloat() {
  if (timeClient.isTimeSet()) TimeNow = timeClient.getEpochTime();
  if (DateTime.isTimeValid()) TimeNow = DateTime.getTime();
  if (TimeNow) return ((float)TimeNow);
  return ((float)(esp_timer_get_time() / 1000000));
}

////////////////////
// Call HealthCheck.io
// healthy less than or equal to zero is a failure, postive integer is success.
#ifdef HEALTHCHECKS_SERVER
int RemoteHealthCall(int healthy, const char *status, const char *agentstring) {
  if (!NetworkGood()) return (-86);
  if (healthy > 0) {
    // We're reporting success, so skip if untimely
    float age = getNowFloat() - LastHealthCheckSuccess;
    if (age < HealthCheckInterval) {
      log_i("Skip HC, only %0f secs since last success.", age);
      return (1);
    }
  }

  int httpResponseCode = 99;
  //WiFiClientSecure *client = SecureClient;

  LastHealthCheck = getNow();  // Reset counter as we're at least attempting to check
  // If this is an unhealthy report, force the loop to send the next auto-check sooner
  if (!healthy) LastHealthCheck = TimeNow - HealthCheckInterval + 120;

  const char *pingURL = RemoteHealthCallRegister();
  if (!pingURL || !strlen(pingURL)) {
    log_i("We have no pingURL, will not call HealthCheck");
    return (-100);
  }
  String longurl(pingURL);

  // See documentation at https://healthchecks.io/docs/http_api/#log-slug
  if (0 > healthy) longurl += "/log";    // Reporting a log event, does not affect up/down
  if (0 == healthy) longurl += "/fail";  // Reporting a failure

  String httpRequestData = (status && strlen(status)) ? String(status) : uptime_formatter::getUptime();
  if (httpRequestData.length()) log_i("Supplied status '%s'", httpRequestData.c_str());

  {
    // Force scoping so HTTPclient is destroyed before WCS
    HTTPClient http;

    const char *useragent = agentstring;
    if (!useragent || !strlen(useragent)) {
      getMyIP();  // Populates MyIPAddress
      useragent = MyIPAddress;
    }

    ConfigureHTTPClient(http, useragent);
    log_i("HC begin to %s", longurl.c_str());
    int err = http.begin(*client, longurl);
    if (err) {

      // This is a workaround to debug an issue with http.POST
      httpRequestData = "OK";

      auto bytes = httpRequestData.length();
      //http.addHeader(ContentType, "application/x-www-form-urlencoded");
      http.addHeader(ContentType, TextPlain);
      // Send HTTP POST request
      log_d("About to POST %d bytes containing '%s' ", bytes, httpRequestData.c_str());
      feed_watchdog();
      httpResponseCode = http.POST((uint8_t *)httpRequestData.c_str(), bytes);
      feed_watchdog();
      if (httpResponseCode < 1) {
        // Total failure
        snprintf(LastWebError, sizeof(LastWebError) - 1, "POSTed %d bytes to HC, got %d",
                 bytes, httpResponseCode);
        log_i("%s", LastWebError);
      }
      if (httpResponseCode > 0) {
        String payloadbuf;
        http.setTimeout(1000);  // Workaround to hanging at getString
        payloadbuf = http.getString();
        if (200 != httpResponseCode) {
          snprintf(LastWebError, sizeof(LastWebError) - 1, "POSTing %d bytes to HC, got HTTP %d '%.63s'.",
                   bytes, httpResponseCode, payloadbuf.c_str());
          log_e("%s", LastWebError);
        }
        if (payloadbuf == "OK (not found)") {
          // Remove our bad cached URL
          log_e("Invalid health check URL %s", longurl.c_str());
          bzero(HealthCheckPingURL, sizeof(HealthCheckPingURL));
          if (SavedNVS.begin(TAG, PREF_RW)) {
            if (SavedNVS.isKey(nvs_healthpingurl)) {
              SavedNVS.remove(nvs_healthpingurl);  // Purge the cached healthurl
              log_w("Deleted HealthCheck URL");
            }
            SavedNVS.end();
            log_i("Removed NVS key for healthurl");
          } else log_w("Failed to remove healthurl from NVS");
          httpResponseCode = -42;
          RemoteHealthCallRegister(true);
        } else if (199 < httpResponseCode && httpResponseCode < 300) {
          LastHealthCheckSuccess = LastHealthCheck = getNow();
          snprintf(LastWebError, sizeof(LastWebError) - 1, "HC Success %d", httpResponseCode);
        }
      } else {
        LastHealthCheck = TimeNow - (HealthCheckInterval / 3);
        snprintf(LastWebError, sizeof(LastWebError) - 1, "HC fail %d %.63s",
                 httpResponseCode, http.errorToString(httpResponseCode).c_str());
        log_e("HC to '%s' %s", longurl.c_str(), LastWebError);
      }
      // Free resources
      http.end();
    } else {
      strncpy(LastWebError, "HC HTTPS failed in begin()", sizeof(LastWebError) - 1);
      log_w("Failed: %s", LastWebError);
      LastHealthCheck = TimeNow - HealthCheckInterval / 5;  // Force a retry, soon
      httpResponseCode = -1 * abs(httpResponseCode);
    }
  }
  return (httpResponseCode);
}
#endif



#ifdef DOHEALTHCHECK
////////////////////
// Call HealthCheck.io to pause/resume a check
int RemoteHealthAPI(const char *longurl) {
  int httpResponseCode = -99;
  bzero(LastHealthCheckResult, sizeof(LastHealthCheckResult));
  if (!longurl || !strlen(longurl)) {
    strncpy(LastHealthCheckResult, "HealthCheck API URL is empty", sizeof(LastHealthCheckResult) - 1);
    log_w("%s", LastHealthCheckResult);
    return (httpResponseCode);
  }
  //WiFiClientSecure *client = SecureClient;
  client->setInsecure();
  {
    // Force scoping so HTTPclient is destroyed before WCS
    HTTPClient http;
    char *myip = getMyIP();
    ConfigureHTTPClient(http, myip);
    log_d("http.begin '%s'", longurl);
    int err = http.begin(*client, longurl);
    if (err) {
      static const char emptyPayload[] = "";
      auto bytes = strlen(emptyPayload);
      http.addHeader("api-key", HEALTHCHECK_KEY_RW);
      http.addHeader("X-Api-Key", HEALTHCHECK_KEY_RW);
      http.addHeader("Content-Type", "text/plain");
      feed_watchdog();
      httpResponseCode = http.POST((uint8_t *)emptyPayload, bytes);
      feed_watchdog();
      log_d("POSTed %d bytes, to '%s' got response code %d", bytes, longurl, httpResponseCode);
      if (httpResponseCode > 0) {
        bzero(LastHealthCheckResult, sizeof(LastHealthCheckResult));
        snprintf(LastHealthCheckResult, sizeof(LastHealthCheckResult) - 1, "API to '%s' got '%s'",
                 longurl, http.getString().c_str());
        if (200 != httpResponseCode) log_w("POSTing to '%s', got HTTP %d '%s'.", longurl, httpResponseCode, LastHealthCheckResult);
        // Free resources
        http.end();
      } else {
        snprintf(LastHealthCheckResult, sizeof(LastHealthCheckResult) - 1,
                 "POST to '%s' failed with %d", longurl, httpResponseCode);
        log_w("%s", LastHealthCheckResult);
        httpResponseCode = -1 * abs(httpResponseCode);
      }
    } else {
      snprintf(LastHealthCheckResult, sizeof(LastHealthCheckResult) - 1,
               "HTTPS begin() failed for '%s'", longurl);
      log_w("%s", LastHealthCheckResult);
    }
  }
  log_i("HC Result: '%s'", LastHealthCheckResult);
  if (connectedMQTT) {
    snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "stats/%s/healthcheck", Where);
    mqttClient.publish(MQTTtopic, 0, false, LastHealthCheckResult);
  }
  return (httpResponseCode);
}
#endif


////////////////
// Simply populate the global MyIPAddress
char *FindMyIP() {
  bzero(MyIPAddress, sizeof(MyIPAddress));
  strncpy(MyIPAddress, WiFi.localIP().toString().c_str(), sizeof(MyIPAddress) - 1);
#ifdef ESP32POE
  if (ETH.localIP().toString().compareTo(zeroip)) {
    strncpy(MyIPAddress, ETH.localIP().toString().c_str(), sizeof(MyIPAddress) - 1);
    WantWiFi = false;
  }
#endif
  return (MyIPAddress);
}


////////////////
// Get my IP address, can also force DHCP attempt
char *getMyIP(boolean forcedhcp) {
#ifndef ESP32POE
  strncpy(MyIPAddress, WiFi.localIP().toString().c_str(), sizeof(MyIPAddress) - 1);
#else
  static unsigned long ethernetRetryTime = 0;

  if (ETH.localIP().toString().compareTo(zeroip)) {
    // IP is not all zeroes.
    strncpy(MyIPAddress, ETH.localIP().toString().c_str(), sizeof(MyIPAddress) - 1);
  } else {
    if (!forcedhcp || ethernetRetryTime > TimeNow) {
      if (forcedhcp) log_e("Won't retry DHCP for another %d seconds", ethernetRetryTime - getNow());
      return (MyIPAddress);
    }
    log_w("No valid IP address found via ETH.localIP(), restarting DHCP");
    feed_watchdog();  // tickle the watchdog
    //ETH.config(IPAddress(0, 0, 0, 0), IPAddress(192, 168, 2, 1), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 1));
    ETH.config((uint32_t)0x00000000, (uint32_t)0x00000000, (uint32_t)0x00000000);
    feed_watchdog();  // tickle the watchdog
    for (int tries = 0; tries < 9 && EthernetConnected < 1; tries++) {
      Serial.print("e");
      do_delay(SleepDuration / 3);
    }

    ethernetRetryTime = TimeNow + 120;
    Serial.println(".");
    if (EthernetConnected > 0) log_i("Ethernet UP");
    strncpy(MyIPAddress, ETH.localIP().toString().c_str(), sizeof(MyIPAddress) - 1);
    log_w("After ethernet reconfig, my IP is %s", MyIPAddress);
  }
#endif
  return (MyIPAddress);
}


char *makeSlug(const char *input, char *output) {
  char *slug = output;
  while (*input != '\0') {
    if (isalpha(*input)) {
      *output = tolower(*input);
      output++;
    } else if (isdigit(*input) || *input == '-' || *input == '_') {
      *output = *input;
      output++;
    }
    input++;
  }
  *output = '\0';  // Null-terminate the output string
  return (slug);
}

#ifdef DOHEALTHCHECK
inline void BuildHealthCheckJsonDocument(JsonDocument &doc) {
  char tags[64];
  char slug[sizeof(Where)];

  doc["name"] = Where;

  snprintf(tags, sizeof(tags), "esp32 %.32s %s", Where, TAG);
  doc["tags"] = tags;
  doc["slug"] = makeSlug(Where, slug);  // Can contain a-z, 0-9, hyphens, _
  doc["channels"] = HEALTHCHECKINTEGRATION;
  doc["manual_resume"] = true;
  doc["grace"] = 86400;
  doc["interval"] = 3600 * 3;
}

inline bool ParseHealthCheckRegistrationResponse(const String &payload) {
  JsonDocument doc;
  JsonDocument filter;
  filter["ping_url"] = true;
  filter["pause_url"] = true;
  filter["resume_url"] = true;
  filter["update_url"] = true;

  DeserializationError err = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
  if (err) {
    log_w("Deserialization error: '%s'", err.c_str());
    return false;
  }

  const char *ping = doc["ping_url"];
  if (ping && strlen(ping)) {
    strncpy(HealthCheckPingURL, ping, sizeof(HealthCheckPingURL) - 1);
  } else {
    log_w("Didn't find a ping_url in '%s'", payload.c_str());
    return false;
  }

  bzero(HealthCheckPauseURL, sizeof(HealthCheckPauseURL));
  const char *pause = doc["pause_url"];
  if (pause && strlen(pause)) {
    strncpy(HealthCheckPauseURL, pause, sizeof(HealthCheckPauseURL) - 1);
    log_i("Set pause url to '%s'", HealthCheckPauseURL);
  } else log_w("Didn't find a pause_url in '%s'", payload.c_str());

  bzero(HealthCheckResumeURL, sizeof(HealthCheckResumeURL));
  const char *resume = doc["resume_url"];
  if (resume && strlen(resume)) {
    strncpy(HealthCheckResumeURL, resume, sizeof(HealthCheckResumeURL) - 1);
    log_i("Set resume url to '%s'", HealthCheckResumeURL);
  } else log_w("Didn't find a resume_url in '%s'", payload.c_str());

  bzero(HealthCheckUpdateURL, sizeof(HealthCheckUpdateURL));
  const char *modify = doc["update_url"];
  if (modify && strlen(modify)) {
    strncpy(HealthCheckUpdateURL, modify, sizeof(HealthCheckUpdateURL) - 1);
    log_i("Set mdodify url to '%s'", HealthCheckUpdateURL);
  } else log_w("Didn't find a modify_url in %s", payload.c_str());

  return true;
}
#endif

////////////////
// Register to healthchecks.io
#ifdef DOHEALTHCHECK
const char *RemoteHealthCallRegister(boolean force) {
  static unsigned int attempts = 0;

  // If we have a URL, return it -- otherwise zero out the buffer
  if (!force && strlen(HealthCheckPingURL) > 5) {
    log_d("Using existing URL %s", HealthCheckPingURL);
    return (HealthCheckPingURL);
  }

  if (!NetworkGood()) {
    log_e("Cannot register, no network attached!");
    return (HealthCheckPingURL);
  }

  if (force) {
    if (strlen(HealthCheckPingURL)) {
      //  Need to do http sendRequest("DELETE") to remove old check!
      RemoteHealthAPI(HealthCheckPauseURL);
    }
    log_w("Forcing new healthurl, deleting cached URL if any");
    bzero(HealthCheckPingURL, sizeof(HealthCheckPingURL));
    if (SavedNVS.begin(TAG, PREF_RW)) {
      if (SavedNVS.isKey(nvs_healthpingurl)) SavedNVS.remove(nvs_healthpingurl);  // Purge the cached healthurl
      SavedNVS.end();
      log_i("Removed NVS key for healthurl");
    }
  } else {
    log_i("Retrieving saved values from NVS");
    readNVS();
    if (strlen(HealthCheckPingURL) > 5) return (HealthCheckPingURL);
    bzero(HealthCheckPingURL, URLBUFSIZE);
  }
  log_d("Registering health check URL");
  if (attempts++ > 3) {
    log_i("We are making attempt #%d, so skipping and returning", attempts);
    return (HealthCheckPingURL);
  }

  log_d("Building JSON");
  JsonDocument doc;
  BuildHealthCheckJsonDocument(doc);
  doc["unique"].add("name");

  logram();

  bzero(BigJSONbuffer, sizeof(BigJSONbuffer));  // empty the buffer
  log_d("Serializing JSON");
  size_t bytes = serializeJson(doc, BigJSONbuffer, sizeof(BigJSONbuffer));
  log_d("New JSON body is %zu bytes, '%s'", bytes, BigJSONbuffer);

  log_i("Registering healthcheck to '%s'", HealthCheckRegistrationServer);
  String payload = doPOST(HealthCheckRegistrationServer, BigJSONbuffer, HEALTHCHECK_KEY_RW);
  log_i("Returned from Registering healthcheck, deserializing");

  if (!ParseHealthCheckRegistrationResponse(payload)) {
    return NULL;
  }

  if (strlen(HealthCheckPauseURL) || strlen(HealthCheckResumeURL) || strlen(HealthCheckUpdateURL)) {
    log_d("Saving changes to healthcheck to NVS");
    writeNVS();
    log_i("Saved changes to healthcheck to NVS");
  }
  return (HealthCheckPingURL);
}
#endif



////////////////
// Update settings for healthchecks.io
#ifdef DOHEALTHCHECK
inline boolean RemoteHealthCallModify() {
  //static unsigned int attempts = 0;

  if (!NetworkGood()) {
    log_e("Cannot update HC, no network attached!");
    return (false);
  }

  if (!strlen(HealthCheckUpdateURL)) {
    log_e("Cannot update HC, no HealthCheckUpdateURL");
    return (false);
  }

  log_d("Building JSON");
  JsonDocument doc;
  BuildHealthCheckJsonDocument(doc);

  bzero(BigJSONbuffer, sizeof(BigJSONbuffer));  // empty the buffer
  log_d("Serializing JSON");
  size_t bytes = serializeJson(doc, BigJSONbuffer, sizeof(BigJSONbuffer));
  log_i("New JSON body is %zu bytes, '%s'", bytes, BigJSONbuffer);

  doPOST(HealthCheckUpdateURL, BigJSONbuffer, HEALTHCHECK_KEY_RW);
  return (LastWebRequestSucceeded);
}
#endif


////////////////
boolean publishError(const char *message, time_t when) {
  log_e("Error: %s", message);
  if (0 == when) when = getNow();

  tm timeinfo = *localtime(&when);
  char timestr[BUFFER_SIZE];
  strftime(timestr, BUFFER_SIZE, "%e %b %H:%M", &timeinfo);
  bzero(PostDataBuffer, sizeof(PostDataBuffer));
  snprintf(PostDataBuffer, sizeof(PostDataBuffer) - 1, "%s at %s", message, timestr);
  feed_watchdog();
  //if (!connectedMQTT) connectToMqttCheck();
  if (connectedMQTT) {
    log_i("Publishing '%s'", PostDataBuffer);
    mqttClient.publish(uniqueID(), 0, false, PostDataBuffer);
    log_d("PublishError done");
    return true;
  }
  //  Fall through, must have failure.
  log_e("Not connected to MQTT server %s (%s) due to %s", MQTTserver, MQTTserverAddress, MQTTReasonText);
  //connectToMqttCheck();
  return false;
}


////////////////
void PostHeapIOTPlotter() {
  if (!NetworkGood()) return;
  if (7 > GetUpMinutes()) {
    log_w("Waiting for more uptime before sending to IOTplotter, only up %lu", UpMinutes);
    return;
  }

  if (NextIOTPlotterHeapPostSeconds) {
    auto waiting = difftime(getNow(), NextIOTPlotterHeapPostSeconds);
    if (waiting > 0) {
      log_d("Waiting for another %.f seconds before sending (TimeNow is %llu, next will be %lu)",
            waiting, (unsigned long long)TimeNow, NextIOTPlotterHeapPostSeconds);
      return;
    }
  }

  NextIOTPlotterHeapPostSeconds = TimeNow + (2 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);

  log_i("Posting heap to iotplotter as %s", NoWheres);

  log_d("Will send to IOTPlotter");
  bzero(PostDataBuffer, sizeof(PostDataBuffer));
  snprintf(PostDataBuffer, sizeof(PostDataBuffer) - 1,
           "0,upMinutes%s,%lu\n0,HeapFree%s,%ld\n0,StackFree%s,%u",
           NoWheres.c_str(), GetUpMinutes(), NoWheres.c_str(), (long)ESP.getFreeHeap(), NoWheres.c_str(), (unsigned int)uxTaskGetStackHighWaterMark(NULL));

  log_d("Sending '%s'", PostDataBuffer);
  // For scoping
  {
    HTTPClient http;
    log_d("http.begin '%s'", IOTPlotterHeapURL);
    int err = http.begin(*CleartextClient, IOTPlotterHeapURL);
    if (0 == err) {
      log_w("HTTPS failure in begin() to '%s'", IOTPlotterHeapURL);
      return;
    } else {
      http.addHeader("Content-Type", FormURLEncoded);
      http.addHeader("api-key", IOTPlotterHeapAPIKey);
      feed_watchdog();
      auto httpResponseCode = http.POST((unsigned char *)PostDataBuffer, strlen(PostDataBuffer));
      feed_watchdog();
      LastPlotterRequestCode = httpResponseCode;
      if (200 == httpResponseCode) {
        log_d("Posted, got response code %d", httpResponseCode);
      } else {
        log_w("Posted '%s' got response code %d", PostDataBuffer, httpResponseCode);

        switch (httpResponseCode) {
          case 429:
            // IOTplotter requested we hold off
            log_w("IOTplotter asked us to hold off via 429 response");
            NextIOTPlotterHeapPostSeconds = getNow() + (4 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
            break;
          case 503:
            log_w("IOTplotter post failed with 503");
            NextIOTPlotterPostTimeSeconds = TimeNow + (3 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
            break;
          default:
            NextIOTPlotterPostTimeSeconds = TimeNow + (2 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
        }

        //String payload = http.getString();
        bzero(LastPlotterError, sizeof(LastPlotterError));
        strncpy(LastPlotterError, http.getString().c_str(), sizeof(LastPlotterError) - 2);
        log_e("Error %d, payload '%s'", httpResponseCode, LastPlotterError);
      }
      feed_watchdog();
    }
    // Free resources
    http.end();
  }
  LastPostHeapTimeSeconds = getNow();
  log_i("Posted heap to iotplotter, got %d", LastPlotterRequestCode);
}


////////////////
inline void force_interval_reboot() {
  unsigned long uphours = GetUpMinutes() / 60;
  if (uphours < HOURS_BETWEEN_FORCED_REBOOTS) return;

  log_e("Uptime of %lu exceeded %d hours", uphours, HOURS_BETWEEN_FORCED_REBOOTS);

  snprintf(LastWebError, sizeof(LastWebError) - 2, "Uphours %lu exceeds %d hours.Stack free %u, heap free %ld",
           uphours,
           HOURS_BETWEEN_FORCED_REBOOTS, (unsigned int)uxTaskGetStackHighWaterMark(NULL), ESP.getFreeHeap());
  writeLastWebErrorToNVS(-42);

  if (NetworkGood() && !DisabledSensor) {
    //mqttClient.setWill(uniqueID(), 1, true, LastWebError);
    // Replaced "Where" with "Room"
    snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
    mqttClient.setWill(MQTTtopic, 1, true, "ghostwhite");  // Mark sensor as vacant on drop
    mqttClient.publish(MQTTtopic, 1, true, "");            // Purge retained
    PostHeapIOTPlotter();                                  // one last push to iotplotter
  }
  ResetRadar();
  delay(333);
  log_w("Rebooting due to uptime");
  ESP.restart();
}


////////////////////
// Make size of files human readable
// source: https://github.com/CelliesProjects/minimalUploadAuthESP32
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

///EOF///
