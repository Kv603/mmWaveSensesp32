#include <humanstaticLite.h>
/// SeeedStudio humanstaticLite Radar
#define RADAR_BAUD 115200
// Which serial port is the mmWave sensor wired to?
#define SENSOR_PORT Serial1

static HumanStaticLite radar = HumanStaticLite((Stream *)&SENSOR_PORT);


//    arrays of data frames according to the user manual. Their function is to turn on or off the Open Underlying Message function.
const unsigned char close_buff[10] PROGMEM = { 0x53, 0x59, 0x08, 0x00, 0x00, 0x01, 0x00, 0xB5, 0x54, 0x43 };  //switch off Open Underlying Message
const unsigned char open_buff[10] PROGMEM = { 0x53, 0x59, 0x08, 0x00, 0x00, 0x01, 0x01, 0xB6, 0x54, 0x43 };   //switch on Open Underlying Message
//data frame to request DevID
const unsigned char DevID_buff[10] PROGMEM = { 0x53, 0x59, 0x02, 0xA1, 0x00, 0x01, 0x0F, 0x5F, 0x54, 0x43 };

////////////////   Show the latest bodysign
void DisplayBodySign() {
  if (!UseLedStrip) return;
  LastLedUpdate = TimeNow + 3;
#ifdef LITELED
  updateLedMeter(&strip, (unsigned int)radar.bodysign_val, LED_COUNT);
  log_d("Rendered %u", (unsigned int)radar.bodysign_val);
#endif
}

////////////////
int post_iotplotter(int status, const char *key /*= NULL*/, int value /*= 0*/) {
  int err = 0;
  bool doingBodySign = false;
  int httpResponseCode = -4444;

  if (DETAILMESSAGE == status) {
    log_d("wrong routine posting detailmessage, skipping");
    return -13;
  }

  if (!ForcePlot) {
    double sincelastpost = difftime(getNow(), LastStatusPostTimeSeconds);
    if (sincelastpost < SECONDS_BETWEEN_IOTPLOTTER_CALLS) {
      if (key && strlen(key) > 0) {
        log_d("Too soon to post again, ignoring %d: %s=%d, last posted %.f minutes ago, waiting another %.f seconds.",
              status, key, value, sincelastpost / 60.0, SECONDS_BETWEEN_IOTPLOTTER_CALLS - sincelastpost);
      } else {
        log_d("Too soon to post again, ignoring %d because we last posted %.f minutes ago, waiting another %.f seconds.",
              status, sincelastpost / 60.0, SECONDS_BETWEEN_IOTPLOTTER_CALLS - sincelastpost);
      }
      return err;
    }

    double sincelastdetail = difftime(TimeNow, LastDetailPostTimeSeconds);
    if (sincelastdetail < 10) {
      if (key && strlen(key) > 0) {
        log_d("Too soon to post detail again, ignoring status %d: %s=%d, last detail post was %.f seconds ago.",
              status, key, value, sincelastdetail);
      } else {
        log_d("Too soon to post detail again, ignoring status %d, last detail post was %.f seconds ago.",
              status, sincelastdetail);
      }
      return err;
    }
  }

  // DNS resolution
  IPAddress test;
  log_d("Doing nslookup");
  err = WiFi.hostByName(IOTPlotterDNS, test);
  if (err == 1) {  // success
    log_d("DNS Resolved '%s' to %s successfully", IOTPlotterDNS, test.toString().c_str());
  } else {
    log_w("Failed to resolve %s using %s", IOTPlotterDNS, WiFi.dnsIP().toString().c_str());
    setDNSpublic();
    return err;
  }

  // Clear buffers
  bzero(IotPlotterDataBuffer, sizeof(IotPlotterDataBuffer));

  log_d("Will post to iotplotter");
  if (key && strlen(key) > 0) {
    log_d("Post to iotplotter for status %d, key %s, value %d", status, key, value);
  } else {
    log_d("Post to iotplotter for status %d", status);
  }

  // Build the data buffer using std::string for safety and readability
  std::string buffer;

  if (key && strlen(key) > 0) {
    if (strcmp(key, BodySignString) != 0) {
      // Not a bodysign posting → include bodysign + extra fields
      buffer = "0," + std::string(key) + NoWheres + "," + std::to_string(value) + "\n0,status" + NoWheres + "," + std::to_string(status) + "\n0," + std::string(BodySignString) + NoWheres + "," + std::to_string(BodySignVal) + "\n0,static" + NoWheres + "," + std::to_string(radar.static_val) + "\n0,dynamic" + NoWheres + "," + std::to_string(radar.dynamic_val) + "\n0,radar_messages" + NoWheres + "," + std::to_string(RadarMessageCount);
    } else {
      // Bodysign posting
      buffer = "0," + std::string(key) + NoWheres + "," + std::to_string(value) + "\n0,status" + NoWheres + "," + std::to_string(status) + "\n0,static" + NoWheres + "," + std::to_string(radar.static_val) + "\n0,dynamic" + NoWheres + "," + std::to_string(radar.dynamic_val);
    }
  } else if (status == 0 && difftime(getNow(), LastDetailPostTimeSeconds) > SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES) {
    log_d("Sending null details to zero-out graph line");
    doingBodySign = true;
    buffer = "0,movement_distance_meters" + NoWheres + ",0" + "\n0,motion" + NoWheres + ",0" + "\n0,static_distance_meters" + NoWheres + ",0" + "\n0,approach" + NoWheres + ",0" + "\n0,speed" + NoWheres + ",0" + "\n0,status" + NoWheres + ",0" + "\n0,bodysign" + NoWheres + ",0" + "\n0,radar_messages" + NoWheres + ",0";
    LastDetailPostTimeSeconds = getNow();
  }

  // Fallback if buffer is still empty
  if (buffer.empty()) {
    if (INVALIDBODYSIGN != BodySignVal) {
      doingBodySign = true;
      buffer = "0,status" + NoWheres + "," + std::to_string(status) + "\n0,bodysign" + NoWheres + "," + std::to_string(BodySignVal) + "\n0,radar_messages" + NoWheres + "," + std::to_string(RadarMessageCount);
    } else {
      buffer = "0,status" + NoWheres + "," + std::to_string(status) + "\n0,radar_messages" + NoWheres + "," + std::to_string(RadarMessageCount);
    }
  }

  // Copy to the fixed buffer (other code still requires the global char array)
  strncpy(IotPlotterDataBuffer, buffer.c_str(), sizeof(IotPlotterDataBuffer) - 1);
  IotPlotterDataBuffer[sizeof(IotPlotterDataBuffer) - 1] = '\0';

  // Skip duplicate status (after building buffer)
  if (!key && !ForcePlot && status == StatusValueLastPosted && difftime(TimeNow, LastStatusPostTimeSeconds) < SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES) {
    log_d("Skip dup status %d", status);
    return 0;
  }

  log_d("Doing HTTP to iotplotter");

  // Scope HTTPClient so it is destroyed before any secure client
  {
    HTTPClient http;
    http.setUserAgent("Mozilla/5.0 (42) Firefox/78.0");

    log_d("http.begin '%s'", IOTPlotterURL);
    err = http.begin(*CleartextClient, IOTPlotterURL);
    if (err == 0) {
      log_e("HTTPS failure in begin() to '%s'", IOTPlotterURL);
      return -11;
    }

    LastStatusPostTimeSeconds = getNow();

    http.addHeader("Access-Control-Request-Headers", "*");
    http.addHeader("Content-Type", FormURLEncoded);
    http.addHeader("api-key", IOTPlotterAPIKey);

    feed_watchdog();
    log_d("Sending '%s' to %s", IotPlotterDataBuffer, IOTPlotterURL);

    httpResponseCode = http.POST((unsigned char *)IotPlotterDataBuffer, strlen(IotPlotterDataBuffer));
    feed_watchdog();
    if (httpResponseCode == 200) {
      ForcePlot = false;
      LastStatusPostTimeSeconds = LastStatusPostSuccessTimeSeconds = TimeNow;
      if (doingBodySign) {
        LastBodySignPostSuccessTimeSeconds = TimeNow;
      }
      log_d("Posted to iotplotter successfully");
      NextIOTPlotterPostTimeSeconds = SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES + TimeNow;

      if (key && strlen(key) > 0) {
        log_i("Posted %s=%d to iotplotter, got response code %d", key, value, httpResponseCode);
      } else {
        log_d("Posted Radar to iotplotter, got response code %d", httpResponseCode);
      }
      log_d("Local IP address is %s", FindMyIP());
      do_delay(SleepDuration);
    } else {
      log_w("Posted, got response code %d", httpResponseCode);
    }

    // Handle rate limiting / errors
    switch (httpResponseCode) {
      case 429:
        ForcePlot = false;
        log_w("IOTplotter asked us to hold off via 429 response");
        NextIOTPlotterPostTimeSeconds = 2 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES + TimeNow;
        break;
      case 503:
        ForcePlot = false;
        log_w("IOTplotter post failed with 503");
        NextIOTPlotterPostTimeSeconds = 3 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES + TimeNow;
        break;
      default:
        NextIOTPlotterPostTimeSeconds = SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES + TimeNow;
    }

    feed_watchdog();
    strncpy(IotPlotterResponseString, http.getString().c_str(), sizeof(IotPlotterResponseString) - 1);
    IotPlotterResponseString[sizeof(IotPlotterResponseString) - 1] = '\0';

    static char result[256];
    if (httpResponseCode < 1) {
      snprintf(result, sizeof(result) - 1, "POST to iotplotter failure with %d", httpResponseCode);
      log_e("Failed with %d result %s", httpResponseCode, IotPlotterResponseString);
      LastPlotterRequestCode = httpResponseCode;
      strncpy(LastPlotterError, result, sizeof(LastPlotterError) - 1);
    } else {
      log_d("HTTP Response code: %d", httpResponseCode);
      do_delay(SleepDuration);

      snprintf(result, sizeof(result) - 1, "iotplotter got %d with %s",
               httpResponseCode, IotPlotterResponseString);

      if (httpResponseCode >= 200 && httpResponseCode < 300) {
        StatusValueLastPosted = status;
        LastStatusPostTimeSeconds = getNow();
        log_d("%s", result);
      } else {
        log_e("Failed to post '%s' got %d: '%s'", IotPlotterDataBuffer, httpResponseCode, IotPlotterResponseString);
        LastPlotterRequestCode = httpResponseCode;
        strncpy(LastPlotterError, IotPlotterResponseString, sizeof(LastPlotterError) - 1);
      }
    }

    http.end();
  }  // HTTPClient goes out of scope here

  feed_watchdog();
  if (httpResponseCode > 299) log_w("Errored post to iotplotter with %d", httpResponseCode);
  else log_d("Finished post to iotplotter with %d", httpResponseCode);

  if (httpResponseCode == 200 || httpResponseCode < 1) {
    return httpResponseCode;
  }
  return -httpResponseCode;  // failure
}


inline void statusToIOTPlotterPostIfNeeded() {
  if (LastStatusPostTimeSeconds < 1) {
    log_d("Have never posted to IOTplotter!");
    LastStatusPostTimeSeconds = 1;
    post_iotplotter(radar.radarStatus, NULL, 0);
  } else {
    auto elapsed = difftime(TimeNow, LastStatusPostTimeSeconds);
    if (elapsed > SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES) {
      log_d("Have not posted to IOTplotter in %.f minutes", elapsed / 60);
      if (radar.radarStatus) {
        post_iotplotter(radar.radarStatus, NULL, 0);
      } else if (elapsed > ForcedRadarIotPlotterInterval) post_iotplotter(radar.radarStatus, NULL, 0);
    }
  }
}


///////////////////////////////////////////////////////
//
void DoHumanStaticPresenceLite() {
  boolean handled = false;

  log_d("Reading Radar...");
  radar.HumanStatic_func(true);  // attempt to get bodysign
#if CORE_DEBUG_LEVEL > 3
  if (radar.radarStatus) Serial.print(", ");
  Serial.print(radar.radarStatus);
#endif

  if (ForcePublish && (HUMANPARA == radar.radarStatus)) handleForcePublish();

  if (radar.radarStatus == 0x00) {
    handleNoRadarMessage();
  } else {
    handled = handleRadarStatus(handled);
  }

  handleBodySignPublishing();
  statusToIOTPlotterPostIfNeeded();
}

inline boolean isTimeForNextIOTPlotter() {
  return NextIOTPlotterPostTimeSeconds < getNow();
}

////////////////////////////////
// Force publish bodysign immediately
inline void handleForcePublish() {
  BodySignVal = radar.bodysign_val;
  log_w("Forced to publish bodysign=%d", BodySignVal);
  publish_bodysign_MQTT(BodySignVal, true);

  (BodySignVal > BodySignBaseVal) ? publish_presence(1) : publish_presence(0);

  publish_bodysign_Influx_MQTT(BodySignVal, true);
  FloorplanBodySign(BodySignVal, true);

  LastRadarMessageHandledTime = TimeNow;
  ForcePublish = false;
}


/////////////////////////////
void handleNoRadarMessage() {
  auto timesince = difftime(TimeNow, LastRadarMessageHandledTime);
  snprintf(RadarMessage, sizeof(RadarMessage) - 1, "No status to report for past %.f seconds", timesince);

  if (timesince > ForcedRadarNoMessagePublishInterval) {
    BodySignVal = 0;  // No message, so treat it as zero bodysign
    log_i("Radar has been quiet for %lu seconds, bodysign is %d, publishing as %d",
          (unsigned long)timesince, radar.bodysign_val, BodySignVal);
    publish_bodysign_MQTT(BodySignVal, true);
    post_iotplotter(radar.radarStatus, BodySignString, BodySignVal);
    publish_bodysign_Influx_MQTT(BodySignVal, true);

    publishEmptyFloorplan();
    publish_presence(0);
    LastRadarMessageHandledTime = getNow();
    ForcePublish = false;
  }
}

inline void publishEmptyFloorplan() {
  // Replaced "Where" with "Room"
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
  mqttClient.publish(MQTTtopic, 0, false, "");
}

inline int RadarStatus() {
  return radar.radarStatus;
}

boolean handleRadarStatus(boolean handled) {
  RadarMessageCount++;
  LastRadarMessageTime = TimeNow;

  switch (radar.radarStatus) {
    case SOMEONE:
      handled = handleRadarSomeone();
      DecayBodySign(true);  // Don't decrease brightness
      break;
    case NOONE:
      // No person is here
      handled = handle_radar_noone();
      DecayBodySign(false);  // Decrease brightness
      break;
    case NOTHING:
      // No Message received
      handled = handle_radar_nothing();
      DecayBodySign(false);  // Decrease brightness
      break;
    case SOMEONE_STOP:
      log_d("Someone stop");
      handled = handle_radar_stop();
      DecayBodySign(true);
      break;
    case SOMEONE_MOVE:
      handled = handle_radar_move();
      DecayBodySign(true);  // Don't decrease brightness
      break;
    case HUMANPARA:
      DisplayBodySign();  // HUMANPARA is the only message that might have a new value
      handled = handle_radar_humanpara();
      break;
    case SOMEONE_CLOSE:
      handled = handleRadarApproach("Someone is closing", 6, 10);
      DecayBodySign(true);
      break;
    case SOMEONE_AWAY:
      DecayBodySign(false);
      handled = handleRadarApproach("Someone is staying away", 5, -10);
      break;
    case DETAILMESSAGE:
      handled = handle_radar_detail();
      break;
    default:
      handled = handleUnknownRadarStatus();
      break;
  }

  return finalizeRadarHandling(handled);
}

boolean handleRadarSomeone() {
  (BodySignVal > BodySignBaseVal) ? publish_presence(1) : publish_presence(0);
  strcpy(RadarMessage, "Someone is here.");
  log_d("%s", RadarMessage);
  if (BodySignVal < 1) BodySignVal = 2;
  FloorplanBodySign(BodySignVal);
  post_iotplotter(radar.radarStatus, "person", 10);
  return true;
}

boolean handleRadarApproach(const char *message, int bodySign, int plotterValue) {
  strcpy(RadarMessage, message);
  log_d("%s", RadarMessage);

  if (isTimeForNextIOTPlotter()) {
    post_iotplotter(radar.radarStatus, "approach", plotterValue);
  } else {
    logTimeToNextIOTPlotter();
  }

  if (BodySignVal < 1) BodySignVal = bodySign;
  return true;
}

inline void logTimeToNextIOTPlotter() {
#if CORE_DEBUG_LEVEL > 2
  log_d("Waiting for another %ld seconds before sending message (TimeNow is %llu, waiting until %llu).",
        (long)(NextIOTPlotterPostTimeSeconds - TimeNow),
        (unsigned long long)TimeNow,
        (unsigned long long)NextIOTPlotterPostTimeSeconds);
#endif
}

boolean handleUnknownRadarStatus() {
  delay(100);
  snprintf(RadarMessage, sizeof(RadarMessage) - 1, "Unknown status %d", radar.radarStatus);
  log_w("%s", RadarMessage);

  if (isTimeForNextIOTPlotter()) {
    post_iotplotter(radar.radarStatus, NULL, 0);
  } else {
    logTimeToNextIOTPlotter();
  }
  return true;
}

boolean finalizeRadarHandling(boolean handled) {
  if (handled) {
    LastRadarMessageHandledTime = TimeNow;
  } else if (radar.radarStatus && radar.radarStatus != DETAILMESSAGE && isTimeForNextIOTPlotter()) {
    log_d("Sending fallback status iotplotter");
    post_iotplotter(radar.radarStatus, NULL, 0);
    handled = true;
    LastRadarMessageHandledTime = TimeNow;
  }
  return handled;
}

inline void publishBodySignIfNeeded() {
  if (LastBodySignInfluxMQTTTimeSeconds < TimeNow - InfluxMQTTPublishInterval)
    publish_bodysign_Influx_MQTT(BodySignVal, true);
  if (LastBodySignFloorplanTimeSeconds < 1) {
    FloorplanBodySign(BodySignVal, true);
  } else if (LastBodySignFloorplanTimeSeconds < TimeNow)
    FloorplanBodySign(BodySignVal);

  if (difftime(TimeNow, LastBodySignMQTTTimeSeconds) > 2 * MQTTPublishInterval)
    publishMQTT();
}

inline void handleBodySignPublishing() {
  if (INVALIDBODYSIGN == BodySignVal) {
    log_d("%s", NoBodySignWarningMessage);
  } else {
    publishBodySignIfNeeded();
  }
}


inline void publishMQTT() {
  if (!DisabledSensor && LastBodySignMQTTTimeSeconds > 0) {
    auto elapsed = difftime(TimeNow, LastBodySignMQTTTimeSeconds);
    log_i("Forcing publish to MQTT, last publish was %.f minutes ago.", elapsed / 60);
  } else {
    log_i("Forcing publish to MQTT, last publish was never");
  }
  publish_bodysign_MQTT(BodySignVal, true);

  if (LastBodySignFloorplanTimeSeconds < TimeNow - SECONDS_BETWEEN_FLOORPLAN_PUBLISHES) {
    FloorplanBodySign(BodySignVal, true);
  }

  if (LastBodySignInfluxMQTTTimeSeconds < TimeNow - InfluxMQTTPublishInterval) {
    publish_bodysign_Influx_MQTT(BodySignVal, true);
  }

  ForcePublish = false;
}






////////////////////////
// Human Movement Parameters: human movement amplitude values.
//The Human Movement Parameters is 0 when no one is in the space, 1-5 when someone is present and stationary,
//and 2-100 when the body is in motion (the greater the motion amplitude the closer the body motion parameter is).
//
inline boolean handle_radar_humanpara() {
  boolean handled = false;
  boolean mustsend = false;          // forces publication.
  BodySignVal = radar.bodysign_val;  // Capture latest result of HUMANPARA
  BodySignValCaptureTime = getNow();
  if (BodySignVal < BodySignBaseVal) {
    log_w("bodysign low-water dropped from %d to %d", BodySignBaseVal, BodySignVal);
    BodySignBaseVal = BodySignVal;
  }
  if (BodySignVal > BodySignPeakVal) {
    log_i("BodySign highwater was %d, is now %d", BodySignPeakVal, BodySignVal);
    BodySignPeakVal = BodySignVal;
    publish_presence(2);
    mustsend = true;
  }

  if (LastFloorplanBodySignVal != BodySignVal
      && difftime(TimeNow, LastBodySignFloorplanTimeSeconds) > SECONDS_BETWEEN_FLOORPLAN_PUBLISHES) {
    log_d("Publishing %d to floorplan", BodySignVal);
    FloorplanBodySign(BodySignVal);
  }
  if (LastPostedBodySignVal != BodySignVal) {
    log_d("Bodysign transitioned from %d to %d", LastPostedBodySignVal, BodySignVal);
    mustsend = true;  // If our new bodysign is now zero, force publish
  }
  bzero(RadarMessage, sizeof(RadarMessage));
  snprintf(RadarMessage, sizeof(RadarMessage) - 1, "Human parameter bodysign %d", BodySignVal);
  if (mustsend || LastPostedBodySignVal != BodySignVal) {
    publish_bodysign_MQTT(BodySignVal, mustsend);
    FloorplanBodySign(BodySignVal, mustsend);
    publish_bodysign_Influx_MQTT(BodySignVal, mustsend);
    handled = true;
    if (mustsend || NextIOTPlotterPostTimeSeconds < TimeNow) {
      log_d("HumanPara BodySign=%d, posting to iotplotter", BodySignVal);
      post_iotplotter(radar.radarStatus, BodySignString, BodySignVal);
      LastPostedBodySignVal = BodySignVal;
      LastBodySignPostTimeSeconds = getNow();
      log_d("HumanPara BodySign=%d, sending to MQTT", BodySignVal, mustsend);
    }
  } else log_d("HumanPara BodySign=%d skipping duplicate value for now", BodySignVal);

  if (mustsend) (BodySignVal > BodySignBaseVal) ? publish_presence(1) : publish_presence(0);


  return handled;
}

///////////////////////
inline boolean handle_radar_nothing() {
  boolean handled = false;
  strcpy(RadarMessage, "No human activity message");
  log_d("%s", RadarMessage);
  if (NextIOTPlotterPostTimeSeconds > getNow()) {
    log_d("Waiting for another %ld minutes before sending NOTHING message (TimeNow is %llu, waiting until %llu).",
          (long)(NextIOTPlotterPostTimeSeconds - TimeNow) / 60, (unsigned long long)TimeNow, (unsigned long long)NextIOTPlotterPostTimeSeconds);
  } else post_iotplotter(radar.radarStatus, "motion", 0);
  handled = true;
  publish_presence(-1);
  do_delay(SleepDuration * 3);
  if (INVALIDBODYSIGN == BodySignVal) log_d("%s", NoBodySignWarningMessage);
  else {
    // We have a bodysign
    if (BodySignlastMQTT != BodySignVal) {
      log_i("Bodysign went from %d to %d, publishing to %s", BodySignlastMQTT, BodySignVal, MQTTserver);
      publish_bodysign_MQTT(BodySignVal, false);
      FloorplanBodySign(BodySignVal);
      publish_bodysign_Influx_MQTT(BodySignVal, false);
    } else if (difftime(TimeNow, LastBodySignMQTTTimeSeconds) > SECONDS_BETWEEN_MQTT_BODYSIGNPUBLISHES) {
      log_i("Bodysign was %d, is %d, last published %llu, sending to %s (%s)",
            BodySignlastMQTT, BodySignVal, (unsigned long long)LastBodySignMQTTTimeSeconds, MQTTserver, MQTTserverAddress);
      publish_bodysign_MQTT(BodySignVal, false);
      FloorplanBodySign(BodySignVal);
    }
  }
  return handled;
}

////////////////////////
inline boolean handle_radar_noone() {
  boolean handled = false;
  BodySignVal = 0;  // Nobody seen, force BodySign to zero.
  strcpy(RadarMessage, "Nobody here.");
  log_d("%s", RadarMessage);
  post_iotplotter(radar.radarStatus, "person", BodySignVal);
  handled = true;
  publish_presence(0);
  log_d("Sending bodysign_val %d to MQTT", BodySignVal);
  if (BodySignlastMQTT != BodySignVal) {
    log_i("Bodysign went from %d to %d, publishing to %s", BodySignlastMQTT, BodySignVal, MQTTserver);
    publish_bodysign_MQTT(BodySignVal, false);
    FloorplanBodySign(BodySignVal);
  } else if (difftime(getNow(), LastBodySignMQTTTimeSeconds) > SECONDS_BETWEEN_MQTT_BODYSIGNPUBLISHES) {
    log_i("Bodysign went from %d to %d, last published %llu, sending to %s (%s)",
          BodySignlastMQTT, BodySignVal, (unsigned long long)LastBodySignMQTTTimeSeconds, MQTTserver, MQTTserverAddress);
    publish_bodysign_MQTT(BodySignVal, false);
    FloorplanBodySign(BodySignVal);
  }
  FloorplanBodySign(BodySignVal);
  return handled;
}

////////////////////////
inline boolean handle_radar_stop() {
  strcpy(RadarMessage, "Someone stop");
  log_d("%s", RadarMessage);
  log_d("Sending presence(1) to iotplotter (%s)", RadarMessage);
  if (BodySignVal < 1) BodySignVal = 1;  // Force bodysign for a stationary person.
  if (NextIOTPlotterPostTimeSeconds > TimeNow) {
    log_d("Waiting for another %ld minutes before sending STOP message (TimeNow is %llu, waiting until %llu).",
          (long)(NextIOTPlotterPostTimeSeconds - TimeNow) / 60, (unsigned long long)TimeNow, (unsigned long long)NextIOTPlotterPostTimeSeconds);
  } else post_iotplotter(radar.radarStatus, "motion", 2);
  log_d("Sending presence(1) to MQTT");
  publish_presence(1);
  FloorplanBodySign(BodySignVal, true);
  return true;
}


////////////////////////
inline boolean handle_radar_move() {
  strcpy(RadarMessage, "Someone moving");
  log_d("%s", RadarMessage);
  if (BodySignVal < 1) BodySignVal = 7;  // Force bodysign for a moving person.
  log_d("Sending presence(2) to iotplotter (%s)", RadarMessage);
  if (NextIOTPlotterPostTimeSeconds > TimeNow) {
    log_d("Waiting for another %ld minutes before sending MOTION message (TimeNow is %llu, waiting until %llu).",
          (long)(NextIOTPlotterPostTimeSeconds - TimeNow) / 60, (unsigned long long)TimeNow, (unsigned long long)NextIOTPlotterPostTimeSeconds);
  } else post_iotplotter(radar.radarStatus, "motion", 10);
  log_d("Sending presence(2) to MQTT");
  publish_presence(2);
  FloorplanBodySign(BodySignVal, true);
  return true;
}


////////////////////////
inline boolean handle_radar_detail() {
  boolean handled = false;

#if CORE_DEBUG_LEVEL > 1
  Serial.print("Spatial static values: ");
  Serial.println(radar.static_val);
  Serial.print("Distance to stationary object: ");
  Serial.print(radar.dis_static);
  Serial.println(" m");
  delay(100);
  Serial.print("Spatial dynamic values: ");
  Serial.println(radar.dynamic_val);
  delay(100);
  Serial.print("Distance from the movement object: ");
  Serial.print(radar.dis_move);
  Serial.println(" m");
  delay(100);
  Serial.print("Speed of moving object: ");
  Serial.print(radar.speed);
  Serial.println(" m/s");
#endif
  bzero(RadarMessage, sizeof(RadarMessage));
  snprintf(RadarMessage, sizeof(RadarMessage) - 1, "Detail message %.1f/%.2f", radar.dis_static, radar.dis_move);
  log_d("Sending distance speed to iotplotter");

  if ((radar.dis_move || radar.dis_static || radar.speed || radar.dynamic_val)) {
    if (NextIOTPlotterPostTimeSeconds > GetUpMinutes()) {
      log_d("Waiting for another %ld minutes before sending DETAIL message (TimeNow is %lu, waiting until %lu).",
            (long)(NextIOTPlotterPostTimeSeconds - TimeNow) / 60, UpMinutes, NextIOTPlotterPostTimeSeconds);
    } else post_Detail_iotplotter(radar.radarStatus, NULL, BodySignVal);
    handled = true;
  }
  return handled;
}

///////////////////////////////////////////////////////////
inline void StartHumanStaticPresenceLite() {

  log_w("Starting up Radar");
  // Using the built-in RX1 and TX1 ports
  SENSOR_PORT.begin(RADAR_BAUD, SERIAL_8N1, RX1, TX1, false);
  log_w("Started radar serial on pins %d/%d at %d baud", RX1, TX1, RADAR_BAUD);

  delay(100);
  // auto port = testSerial((Stream *) &SENSOR_PORT);

  ResetRadar();
  delay(SleepDuration);
  log_w("Checking radar mode");
  radar.checkSetMode_func(DevID_buff, 10, false);  // Array, ArrayLen, RepeatFlag
  delay(500);
  log_w("Checking Radar data");
  radar.recvRadarBytes();  //Receive radar data and start processing

#if CORE_DEBUG_LEVEL > 2
  radar.showData();  //Serial port prints a set of received data frames
#endif
  log_i("BodySign=%d", radar.bodysign_val);
  delay(200);  //Add time delay to avoid program jam
}

inline void PutRadarDetailsIntoPostDataBuffer() {
  // Clear the buffer first
  // Only post if there's actually something meaningful to report
  if (!radar.dis_move && !radar.dis_static && !radar.speed && !radar.dynamic_val) {
    log_d("No details worth publishing.");
    return;
  }

  log_d("Posting detail message for movement at %.2f m or static person at %.1f m",
        radar.dis_move, radar.dis_static);

  // Build the detail message using std::string for safety and readability
  std::string buffer =
    "0,movement_distance_meters" + NoWheres + "," + std::to_string(radar.dis_move) +    // 2 decimal places
    "\n0,static_distance_meters" + NoWheres + "," + std::to_string(radar.dis_static) +  // 1 decimal place
    "\n0,approach" + NoWheres + ",0"
                                "\n0,speed"
    + NoWheres + "," + std::to_string(radar.speed) +  // 3 decimal places
    "\n0,static" + NoWheres + "," + std::to_string(radar.static_val) + "\n0,dynamic" + NoWheres + "," + std::to_string(radar.dynamic_val) + "\n0,radar_messages" + NoWheres + "," + std::to_string(RadarMessageCount);

  // Copy the result into the fixed-size buffer
  strncpy(PostDataBuffer, buffer.c_str(), sizeof(PostDataBuffer) - 1);
  PostDataBuffer[sizeof(PostDataBuffer) - 1] = '\0';
}



/////////////////////////////////////
inline void SetRadarMode() {
  log_w("Setting Radar mode (this could take over a full minute)");
  auto started = getNow();
  radar.checkSetMode_func(open_buff, 10, false);
  LastRadarModeSetTime = getNow();
  auto duration = difftime(LastRadarModeSetTime, started);
  log_w("Setting radar mode took %.f seconds");
}


inline void ResetRadar() {
  log_w("Resetting radar!");
#ifdef LITELED
  if (UseLedStrip) strip.clear(true);
#endif
  radar.reset_func();
}

///////////////////////////////
inline void handleRadarMode(boolean force) {
  if (force || difftime(getNow(), LastRadarMessageTime) > RadarResetInterval) {
    // We haven't seen anything from the radar in a while.
    auto started = getNow();
    ResetRadar();
    auto duration = difftime(getNow(), started);
    log_i("Completed radar reset in %.f seconds");
    feed_watchdog();
    //delay(2000);
  }
  SetRadarMode();
  LastRadarModeSetTime = getNow();
}



///////////////////
boolean FloorplanBodySign(int val, boolean force) {
  boolean success = false;
  auto now = getNow();
  if (LastFloorplanBodySignVal == val && abs(LastBodySignFloorplanTimeSeconds - now) < 2) {
    // even if called with force, we won't send same value twice in the same second.
    log_d("Too soon to re-report identical value %d ", val);
    return false;
  }

  if (DisabledSensor) {
    WarnDisabled();
    return false;
  }

  if (!force) {
    if (LastFloorplanBodySignVal == val) return false;
    if (LastBodySignFloorplanTimeSeconds == now) {
      log_d("Too soon");
      return false;
    }
  }

  // Publish for HomeSeer's sensor/$$FLOOR:/$$ROOM/:$$PARENTNAME:/$$NAME:=$$VALUE:
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/%s/%s/%s", Topic_HomeSeer, Floor, Room, Where);
  feed_watchdog();
  bzero(MQTTreport, sizeof(MQTTreport));
  snprintf(MQTTreport, sizeof(MQTTreport) - 1, "%s=%d", BodySignString, (unsigned char)val);
  success = mqttClient.publish(MQTTtopic, 0, false, MQTTreport);
  feed_watchdog();
  if (success) log_d("Published '%s' (%u) to topic '%s'", MQTTreport, val, MQTTtopic);
  else log_i("Failed to publish '%s' (%u) to topic '%s'", MQTTreport, val, MQTTtopic);

  // Publish to floorplan
  bzero(MQTTtopic, sizeof(MQTTtopic));
  // Replaced "Where" with "Room"
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/mmwave_%s/mmwave", Topic_Floorplan, Room);
  feed_watchdog();
  bzero(MQTTreport, sizeof(MQTTreport));
  //snprintf(MQTTreport, sizeof(MQTTreport) - 1, "#0000%hhX", (unsigned char)(val * 2.5));
  snprintf(MQTTreport, sizeof(MQTTreport) - 1, "%d", (unsigned char)val);
  success = mqttClient.publish(MQTTtopic, 0, false, MQTTreport);
  feed_watchdog();
  if (success) {
    log_d("Published '%s' (%u) to topic '%s'", MQTTreport, val, MQTTtopic);
    LastFloorplanBodySignVal = val;
    LastBodySignFloorplanTimeSeconds = now;
  } else log_w("Failed to publish '%s' to topic '%s'", MQTTreport, MQTTtopic);
  return success;
}


///////////////////
int post_Detail_iotplotter(int status, const char *key, int value) {
  int err = 0;
  if (key && strlen(key)) {
    auto lastdetail = difftime(getNow(), LastDetailPostTimeSeconds);
    if (lastdetail < SECONDS_BETWEEN_IOTPLOTTER_CALLS) {
      log_i("Too soon to post again, ignoring %d: %s=%d because it's been %.f minutes.",
            status, key, value, lastdetail / 60);
      return (0);
    }
  }
  auto laststatus = difftime(getNow(), LastStatusPostTimeSeconds);
  if (laststatus < SECONDS_BETWEEN_IOTPLOTTER_CALLS) {
    log_i("Too soon to post status %d, it's been %.f minutes", status, laststatus / 60);
    return (0);
  }

  IPAddress test;
  log_d("Doing nslookup");
  err = WiFi.hostByName(IOTPlotterDNS, test);
  if (1 == err) {  //success
    log_d("DNS Resolved '%s' to %s successfully", IOTPlotterDNS, test.toString().c_str());
  } else {
    log_w("Failed to resolve %s using %s", IOTPlotterDNS, WiFi.dnsIP().toString());
    setDNSpublic();
    return (-53);
  }

  LastIOTPlotterAttemptTime = TimeNow;

  static char result[256];
  bzero(result, sizeof(result));

  log_d("Will post Detail to iotplotter");
  PutRadarDetailsIntoPostDataBuffer();

  log_d("Doing HTTP to iotplotter");
  // Force scoping so HTTPclient is destroyed before WifiClientSecure client
  HTTPClient http;
  http.setUserAgent("Mozilla/5.0 (42) Firefox/78.0");
  log_d("http.begin '%s'", IOTPlotterURL);
  err = http.begin(*CleartextClient, IOTPlotterURL);
  if (0 == err) {
    log_e("HTTPS failure in begin() to '%s'", IOTPlotterURL);
    return (-42);
  } else {
    log_d("POSTing %s\n", PostDataBuffer);
    // http.setTimeout(HTTP_TIMEOUT_DATA);              // Data timeout
    // http.setConnectTimeout(HTTP_HANDSHAKE_TIMEOUT);  // Setup timeout
    // log_i("Set main timeout to %d seconds, and handshake timeout to %d seconds", HTTP_TIMEOUT_DATA, HTTP_HANDSHAKE_TIMEOUT);
    //http.addHeader("User-Agent", VERSION);
    http.addHeader("Access-Control-Request-Headers", "*");
    http.addHeader("Content-Type", FormURLEncoded);
    http.addHeader("api-key", IOTPlotterAPIKey);

    feed_watchdog();
    log_d("Sending '%s' to %s", PostDataBuffer, IOTPlotterURL);

    auto httpResponseCode = http.POST((unsigned char *)PostDataBuffer, strlen(PostDataBuffer));
    feed_watchdog();
    if (200 == httpResponseCode) {
      log_d("Posted to iotplotter successfully");
      LastDetailPostTimeSeconds = LastDetailPostSuccessTimeSeconds = getNow();
    } else log_w("Posted detail, got response code %d", httpResponseCode);

    switch (httpResponseCode) {
      case 429:
        LastDetailPostTimeSeconds = TimeNow;
        // IOTplotter requested we hold off
        log_w("IOTplotter asked us to hold off via 429 response");
        NextIOTPlotterPostTimeSeconds = TimeNow + (2 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
        break;
      case 503:
        LastDetailPostTimeSeconds = TimeNow;
        log_w("IOTplotter post failed with 503");
        NextIOTPlotterPostTimeSeconds = TimeNow + (3 * SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
        break;
      default:
        LastDetailPostTimeSeconds = TimeNow;
        NextIOTPlotterPostTimeSeconds = TimeNow + (SECONDS_BETWEEN_IOTPLOTTER_STATUSPUBLISHES);
    }
    feed_watchdog();

    if (httpResponseCode < 1) {
      snprintf(result, sizeof(result) - 1, "POST to iotplotter failure with %d", httpResponseCode);
      log_e("Failed with %d", httpResponseCode);
      LastPlotterRequestCode = httpResponseCode;
      strncpy(LastPlotterError, result, sizeof(LastPlotterError) - 2);
    } else {
      log_d("HTTP Response code: %d", httpResponseCode);
      do_delay(SleepDuration);
      //String payload = http.getString();
      snprintf(result, sizeof(result) - 1, "iotplotter got %d with %s", httpResponseCode, http.getString().c_str());
      if (199 < httpResponseCode && httpResponseCode < 300) {
        StatusValueLastPosted = status;
        LastStatusPostTimeSeconds = getNow();
        log_d("%s", result);
      } else {
        log_e("Failed to post '%s' got %s", PostDataBuffer, result);
        LastPlotterRequestCode = httpResponseCode;
        strncpy(LastPlotterError, result, sizeof(LastPlotterError) - 2);
        //writeLastWebErrorToNVS(httpResponseCode);
      }
    }
    // Free resources
    http.end();
  }
  feed_watchdog();
  log_d("Finished post to iotplotter");
  return (0);
}


////////////////
void publish_bodysign_MQTT(int value, boolean force) {
  if (DisabledSensor) {
    WarnDisabled();
    return;
  }
  if (INVALIDBODYSIGN == value) {
    log_d("%s", NoBodySignWarningMessage);
    return;
  }
  if (!connectedMQTT) {
    log_w("Cannot publish to MQTT, not connected due to %s", MQTTReasonText);
    connectToMqttCheck();
    do_delay(333);
    if (!connectedMQTT) return;
  } else log_d("Will publish bodysign to MQTT");

  if (difftime(TimeNow, LastBodySignMQTTTimeSeconds) > MQTTPublishInterval) force = true;

  boolean important = false;
  if (BodySignlastMQTT != value) important = true;

  if (force || important) {
    feed_watchdog();
    snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "sensor/%s/%s", Where, BodySignString);
    bzero(MQTTreport, sizeof(MQTTreport));

    sprintf(MQTTreport, "%d", value);
    log_d("Publishing bodysign=%d to '%s'", value, MQTTtopic);

    if (mqttClient.publish(MQTTtopic, important, important, MQTTreport)) {
      log_d("Published %d to topic '%s' on %s (%s)", value, MQTTtopic, MQTTserver, MQTTserverAddress);
      LastBodySignMQTTTimeSeconds = TimeNow;
      BodySignlastMQTT = value;
    } else {
      log_e("Failed to publish '%s' to '%s'", MQTTreport, MQTTtopic);
      snprintf(MQTTReasonText, sizeof(MQTTReasonText) - 1, "Failure when publishing to %s", MQTTtopic);
    }
  }
}

////////////////
void publish_bodysign_Influx_MQTT(int value, boolean force) {
  if (DisabledSensor) {
    WarnDisabled();
    return;
  }
  if (INVALIDBODYSIGN == value) {
    log_d("%s", NoBodySignWarningMessage);
    return;
  }
  if (!connectedMQTT) {
    log_w("Cannot publish to MQTT, not connected due to %s", MQTTReasonText);
  } else log_d("Ready to publish bodysign to MQTT");

  auto secsSinceLast = difftime(TimeNow, LastBodySignInfluxMQTTTimeSeconds);
  int change = abs(BodySignlastInfluxMQTT - value);

  if (secsSinceLast > InfluxMQTTPublishInterval) force = true;
  else if (secsSinceLast < 15) {
    log_d("Too soon to republish to Influx, only %d seconds elapsed, diff is %d", (int)secsSinceLast, change);
    return;
  }

  if (force) log_i("Forced publish of %d to Influx, last value was %d", value, BodySignlastInfluxMQTT);
  else if (BodySignlastInfluxMQTT == value) {
    log_d("Skipping Influx publish of duplicate %d", value);
    return;
  } else if (change < 4 && BodySignlastInfluxMQTT && value) {
    log_i("Skipping Influx publish of %d as last value was %d (diff %d)", value, BodySignlastInfluxMQTT, change);
    return;
  }

  log_d("Publishing %d to '%s' for Telegraf/InfluxDB, last pub was %d seconds ago", value, Topic_Influx_Occupancy, secsSinceLast);
  bzero(MQTTreport, sizeof(MQTTreport));
  unsigned long long timestamp = (unsigned long long)TimeNow;
  if (timestamp > 1697031337)
    snprintf(MQTTreport, sizeof(MQTTreport) - 1, "occupancy,where=%s,type=mmwave bodysign=%d %llu000000000",
             Where, value, timestamp);
  else {
#ifdef USE_NTP
    log_w("clock is not set, timestamp came back as %llu, discarding timestamp", timestamp);
#endif
    snprintf(MQTTreport, sizeof(MQTTreport) - 1, "occupancy,where=%s,type=mmwave bodysign=%d",
             Where, value);
  }
  if (mqttClient.publish(Topic_Influx_Occupancy, 0, false, MQTTreport)) {
    LastBodySignInfluxMQTTTimeSeconds = TimeNow;
    BodySignlastInfluxMQTT = value;
    log_d("Published bodysign %d to Influx '%s'", value, Topic_Influx_Occupancy);
  } else log_e("Failed Publishing bodysign %d to Influx '%s'", value, Topic_Influx_Occupancy);
}


void BlankRetainedRadarMQTT() {
  log_i("Publishing blank bodysign to MQTT");
  publish_bodysign_MQTT(0, true);
  publish_bodysign_Influx_MQTT(0, true);
  FloorplanBodySign(0, true);
  ForcePublish = false;
  LastBodySignMQTTTimeSeconds = LastBodySignInfluxMQTTTimeSeconds = 0;
  BodySignlastMQTT = BodySignlastInfluxMQTT = INVALIDBODYSIGN;
}

////////////////
//
//
inline void WarnDisabled() {
  log_w("Sensor is disabled '%s'", DisableSensor);
}

////////////////
void publish_presence(int value) {

  if (LastPresencePublishValue == value)
    if (TimeNow - LastPresencePublishTime < 5) return;

  if (DisabledSensor || (-1 == value)) {
    log_d("Reporting lack of data");
    LastPresencePublishValue = -2;
    LastPresencePublishTime = TimeNow + 60;
    mqttClient.publish(MQTTtopic, 1, true, "");

    if (DisabledSensor) WarnDisabled();
    return;
  }

  bzero(MQTTreport, sizeof(MQTTreport));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "sensor/%s/%s", Where, "presence");




  boolean important = true;
  sprintf(MQTTreport, "%d", value);
  log_i("Publishing Presence %d to MQTT '%s'", value, MQTTtopic);
  if (mqttClient.publish(MQTTtopic, 1, important, MQTTreport)) {
    log_d("Published %d to MQTT '%s'", value, MQTTtopic);
    LastPresencePublishTime = TimeNow;
    LastPresencePublishValue = value;
  } else log_w("Failed to publish %d to MQTT topic '%s'", value, MQTTtopic);


  snprintf(MQTTreport, sizeof(MQTTreport) - 1, "occupancy,where=%s,type=mmwave presence=%d", Where, value);

  unsigned long long timestamp = (unsigned long long)TimeNow;
  if (timestamp > 1697031337) {
    log_d("Valid timestamp, building occupancy for Influx");
    snprintf(MQTTreport, sizeof(MQTTreport) - 1, "occupancy,where=%s,type=mmwave presence=%d %llu000000000",
             Where, value, timestamp);
    log_d("Valid timestamp, built occupancy for Influx as '%s'", MQTTreport);
  } else log_w("Clock is not set, timestamp came back as %llu, discarding timestamp", timestamp);

  log_i("Publishing Influx Occupancy  %d to %s", value, Topic_Influx_Occupancy);
  log_d("reporting %s", MQTTreport);
  if (mqttClient.publish(Topic_Influx_Occupancy, 0, false, MQTTreport)) log_d("Published Influx Occupancy %d", value);
  else log_w("Failed to publish %d to Influx topic '%s'", value, Topic_Influx_Occupancy);

  unsigned int val = 0;
  if (value > -1) val = (unsigned int)value;
  // Publish for HomeSeer's sensor/$$FLOOR:/$$ROOM/:$$PARENTNAME:/$$NAME:=$$VALUE:
  bzero(MQTTtopic, sizeof(MQTTtopic));
  snprintf(MQTTtopic, sizeof(MQTTtopic) - 1, "%s/%s/%s/%s", Topic_HomeSeer, Floor, Room, Where);
  bzero(MQTTreport, sizeof(MQTTreport));
  snprintf(MQTTreport, sizeof(MQTTreport) - 1, "%s=%d", PresenceString, (unsigned char)val);
  if (mqttClient.publish(MQTTtopic, 0, false, MQTTreport)) log_d("Published '%s' (%u) to topic '%s'", MQTTreport, val, MQTTtopic);
}

///EOF///