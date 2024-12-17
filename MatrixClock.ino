/*
****************************************************************************
*   Matrix Portal Clock                                                    *
*   Uses Adafruit Matrix Portal S3 (or may work on M4)                    *
*                                                                          *
*   Copyright (C) 2024 by Jeremy Falling except where noted.               *
*                                                                          *
*   This program is free software: you can redistribute it and/or modify   *
*   it under the terms of the GNU General Public License as published by   *
*   the Free Software Foundation, either version 3 of the License, or      *
*   (at your option) any later version.                                    *
*                                                                          *
*   This program is distributed in the hope that it will be useful,        *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*   GNU General Public License for more details.                           *
*                                                                          *
*   You should have received a copy of the GNU General Public License      *
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
****************************************************************************
*/

// matrix portal clock - using a led matrix (with hub75 protocol) Adafruit Matrix Portal S3 (M4 is untested)

// This uses ntp as primary timesource, then ext rtc as backup. both are in utc and converted
//  to a local timezone using a software rtc.
//
// optionally uses mqtt to get messages, which will display as scrolling text. payload should be:
//   {"message": "your message", "repeat": number_of_time_to_repeat(int) (optional), "color", "0x prefixed rgb565 color (optional)"}
//  note that message is limited to 512 chars. Default color is current day/night color and number of repeats is 3.

// uses this font: https://www.1001freefonts.com/homespun.font

// TODO:
// - find block mono-ish space font like this: https://github.com/trip5/Matrix-Fonts



/********************************************************************/

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <FS.h>
//#include <NetworkUdp.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <esp_mac.h>
// adafruit config seems to specify fat in the device config
#include "esp_partition.h"  // to check existing data partitions in Flash memory
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Protomatter.h>
#include <ArduinoJson.h>
#include <ESP32Time.h>
#include <ESPNtpClient.h>
#include <FFat.h>
#include <MQTT.h>
#include <Wire.h>  // For I2C communication
#include <uRTCLib.h>

// fonts
#include "homespun6pt7b.h"
#include "homespun9pt7b.h"

/********************************************************************/
// hardware settings
#define DISPLAY_HEIGHT 32  // Matrix height (pixels) - SET TO 64 FOR 64x64 MATRIX!
#define DISPLAY_WIDTH 64   // Matrix width (pixels)

#define BUTTON_UP 6
#define BUTTON_DOWN 7
#define NEOPIXEL_PIN 4
#define STATUS_LED 13  // staus led next to esp chip

// i2c addrs for rtc and eeprom
uRTCLib external_rtc(0x68);

// dow number to name
const char* days_of_week[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
// const char *days_of_week[] = { "Zondag", "Maandag", "Dinsdag", "Woensdag", "Donderdag", "Vrijdag", "Zaterdag" };

// Uncomment this out to enable ability to flash over the network. Note that some chips do
//   not reset after flashing!
// #define ENABLE_OTA_UPDATES

#define AP_PASSWORD "MatrixPortal"
#define OTA_PASSWORD "MatrixPortal"

/********************************************************************/

const char* PROG_NAME = "Matrix Clock";
const char* VERSION = "1.0.0";
char DEFAULT_DEVICE_NAME[128] = "matrix-clock-";  // used as hostname + end of mac addr

// default values for config. all values should be char or bool
bool mqtt_enabled = false;
char mqtt_server[128];
char mqtt_port[7] = "1883";
char mqtt_queue[128] = "svcs/matrix_clock";
char ntp_pool[128];
char tz_offset[5] = "0";
bool dst_active = false;
char night_mode_start[3] = "8";
char night_mode_end[3] = "20";
int night_mode_start_int = atoi(night_mode_start);
int night_mode_end_int = atoi(night_mode_end);
char hostname[128];

// colors are rgb565 hex
char display_color_day[9] = "0x9CD3";
char display_color_night[9] = "0x6000";
char display_color_error[9] = "0x7a60";

uint16_t display_color_day_hex;
uint16_t display_color_night_hex;
uint16_t display_color_error_hex;

#define BOOTUP_MSG_FONT &homespun6pt7b

#define CLOCK_LAYOUT_DOW_VERT 7
#define CLOCK_LAYOUT_DOW_HORIZ 1
#define CLOCK_LAYOUT_DOW_FONT &homespun6pt7b

#define CLOCK_LAYOUT_TIME_VERT 22
#define CLOCK_LAYOUT_TIME_HORIZ 2
#define CLOCK_LAYOUT_TIME_FONT &homespun9pt7b

#define CLOCK_LAYOUT_DATE_VERT 31
#define CLOCK_LAYOUT_DATE_HORIZ 3
#define CLOCK_LAYOUT_DATE_FONT &homespun6pt7b

#define ERROR_POSITION_VERT 18
#define ERROR_POSITION_FONT &homespun6pt7b

#define DISPLAY_MAX_CHARS_PER_LINE 11

#define NTP_TIMEOUT 5000
#define NTP_UPDATE_INTERVAL 3600

// should not need to be changed, unless not using an adafruit board
#if defined(_VARIANT_MATRIXPORTAL_M4_)  // MatrixPortal M4
uint8_t matrix_rgb_pins[] = { 7, 8, 9, 10, 11, 12 };
uint8_t matrix_addr_pins[] = { 17, 18, 19, 20, 21 };
uint8_t matrix_clock_pin = 14;
uint8_t matrix_latch_pin = 15;
uint8_t matrix_oe_pin = 16;
#else  // MatrixPortal ESP32-S3
uint8_t matrix_rgb_pins[] = { 42, 41, 40, 38, 39, 37 };
uint8_t matrix_addr_pins[] = { 45, 36, 48, 35, 21 };
uint8_t matrix_clock_pin = 2;
uint8_t matrix_latch_pin = 47;
uint8_t matrix_oe_pin = 14;
#endif

#if DISPLAY_HEIGHT == 16
#define NUM_ADDR_PINS 3
#elif DISPLAY_HEIGHT == 32
#define NUM_ADDR_PINS 4
#elif DISPLAY_HEIGHT == 64
#define NUM_ADDR_PINS 5
#endif

// last arg, True, enables double-buffering
Adafruit_Protomatter matrix(
  DISPLAY_WIDTH, 4, 1, matrix_rgb_pins, NUM_ADDR_PINS, matrix_addr_pins,
  matrix_clock_pin, matrix_latch_pin, matrix_oe_pin, true);

bool wifi_first_connection = false;

boolean sync_event_triggered = false;  // True if a time even has been triggered
NTPEvent_t ntpEvent;                   // Last triggered event

// stores msg/errors
JsonDocument internal_message_queue[25];
int internal_message_queue_current_index = -1;
int current_message_repeats = 0;
char* current_message;
uint16_t current_message_color_hex;
bool message_being_displayed = false;

int last_ntp_error_timestamp = 0;
bool ntp_is_synced = false;
bool rtc_failure = false;
bool reboot_required = false;

int16_t text_pos_x;    // Current text position (X)
int16_t text_pos_y;    // Current text position (Y)
int16_t text_pos_min;  // Text pos. (X) when scrolled off left edge
int text_scrolling_repeat_count = 0;
bool config_file_exists = false;

long last_mqtt_connect_attempt = 0;

ESP32Time system_rtc_utc;
ESP32Time system_rtc_localtime;

WiFiClient wifi_client_con;
MQTTClient mqtt_ciient;

WiFiManager wifi_manager_inst;

Adafruit_NeoPixel neo_pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// must be defined globally, then updated later with the right value
char checkbox_html[24] = "type=\"checkbox\"";

WiFiManagerParameter custom_linebreak("<br>");

WiFiManagerParameter custom_hostname("hostname", "Hostname of device (DNS valid characters and no domain)", hostname, 128);

WiFiManagerParameter custom_ntp_pool("ntp_pool", "NTP Pool (if blank then default gateway is used)", ntp_pool, 128);

WiFiManagerParameter custom_tz_offset("tz_offset", "Timezone offset in hours (positive/negative number). Use decimal notation for timezones with fractions of an hour (ie -3.5)", tz_offset, 4);

WiFiManagerParameter custom_dst_active_checkbox("dst_active", "DST is active", "x", 2, checkbox_html, WFM_LABEL_AFTER);

WiFiManagerParameter custom_mqtt_enabled_checkbox("mqtt_enabled", "MQTT enabled", "T", 2, checkbox_html,
                                                  WFM_LABEL_AFTER);

WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT server", mqtt_server, 128);

WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT port", mqtt_port, 6);

WiFiManagerParameter custom_mqtt_queue("mqtt_queue", "MQTT queue", mqtt_queue, 128);

WiFiManagerParameter custom_display_color_info(
  "Display color should be in hex rgb565 (ie: 0x9CD3). The brigher the color, the brighter the display. <br>");

WiFiManagerParameter custom_display_color_day("display_color_day", "Display daytime color", display_color_day, 9);

WiFiManagerParameter custom_display_color_night("display_color_night", "Display nighttime color", display_color_night,
                                                9);

WiFiManagerParameter custom_display_color_error("display_color_error", "Display error color", display_color_error, 9);

WiFiManagerParameter custom_night_mode_start("night_mode_start", "Night mode start (hour)", night_mode_start, 2);

WiFiManagerParameter custom_night_mode_end("night_mode_end", "Night mode end (hour)", night_mode_end, 2);

/********************************************************************/

unsigned long getEpoch() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  return now;
}

String IpAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]);
}

void setNeoPixelColor(int red, int green, int blue) {
  neo_pixel.setPixelColor(0, neo_pixel.Color(red, green, blue));
  neo_pixel.show();
}

int StrToHex(char inStr[]) {
  return (int)strtol(inStr, 0, 16);
}

String padNumber(int number) {
  String padded_number = String(number);

  if (number < 10) {
    padded_number = "0" + padded_number;
  }

  return padded_number;
}

String centerText(String rawText) {
  int padding = (DISPLAY_MAX_CHARS_PER_LINE - rawText.length()) / 2;

  String paddedText = "";
  for (int i = 0; i < padding; i++) {
    paddedText.concat(" ");
  }

  paddedText.concat(rawText);

  return paddedText;
}

void addToMessageQueue(JsonDocument msgDoc, bool isError = false) {
  // add message to queue
  msgDoc["isError"] = isError;
  int array_size = (sizeof(internal_message_queue) / sizeof(internal_message_queue[0]) - 1);
  int queue_full = true;
  for (int i = 0; i < array_size; i++) {
    if (internal_message_queue[i].isNull()) {
      internal_message_queue[i] = msgDoc;
      queue_full = false;
      break;
    }
  }

  if (queue_full) {
    Serial.println("Warning: Internal message queue is full, dropping message...");
  }
}

void displayError(const char* error_msg, int repeat = -1) {
  if (repeat > 0) {
    text_scrolling_repeat_count = repeat;
  } else {
    text_scrolling_repeat_count = 3;
  }

  JsonDocument errDoc;
  errDoc["message"] = error_msg;
  errDoc["repeat"] = text_scrolling_repeat_count;

  addToMessageQueue(errDoc, true);
}

void clearMessageQueue() {
  int array_size = sizeof(internal_message_queue) / sizeof(internal_message_queue[0]);
  JsonDocument tmpDoc;
  for (int i = 0; i < array_size; i++) {
    internal_message_queue[i] = tmpDoc;
  }
  internal_message_queue_current_index = -1;
}

int nextIndexInMessageQueue() {
  // find next message in queue (if exists)
  // internal_message_queue_current_index -1 means no msg
  int array_size = (sizeof(internal_message_queue) / sizeof(internal_message_queue[0]) - 1);
  int nextMessage;

  if (internal_message_queue[0].isNull() && internal_message_queue_current_index == -1) {
    // queue is empty, nothing to do
    return -1;
  }

  // get id of next message in queue
  if (internal_message_queue_current_index == -1) {
    nextMessage = 0;
  } else {
    nextMessage = internal_message_queue_current_index + 1;
  }

  if (nextMessage > array_size || internal_message_queue[nextMessage].isNull()) {
    // reached end of queue, clear it and reset index
    clearMessageQueue();
    return -1;
  } else {
    // there is another msg to process, return id
    return nextMessage;
  }
}

void mqttMessageReceived(String& topic, String& payload) {
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);
  JsonDocument parsed_doc;

  // Test if parsing succeeds
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  if (doc["message"].isNull()) {
    // no msg, return as there is nothing to do
    return;
  }
  // const char* msg = doc["message"].as<const char*>();
  parsed_doc["message"] = doc["message"];

  int repeat = doc["repeat"].as<int>();
  if (repeat < 1) {
    repeat = -1;
  }
  parsed_doc["repeat"] = repeat;

  // uint16_t color_hex = -1;
  const char* color;
  if (!doc["color"].isNull()) {
    parsed_doc["color"] = doc["color"];
    // color = doc["color"].as<const char*>();
    // color_hex = StrToHex(const_cast<char*>(color));
  } else {
    parsed_doc["color"] = "none";
  }

  addToMessageQueue(parsed_doc);
}

void updateExternalRtcTime() {
  // update rtc time from ntp time. as the sys rtc is updated by ntp client, just update use that.
  // note that month returns 0-11, year needs to be two digits
  external_rtc.set(system_rtc_utc.getSecond(), system_rtc_utc.getMinute(), system_rtc_utc.getHour(true),
                   system_rtc_utc.getDayofWeek(), system_rtc_utc.getDay(), system_rtc_utc.getMonth() + 1,
                   system_rtc_utc.getYear() - 2000);
  Serial.println("Updating external RTC");
}

void syncExtRtcToSystemRtc() {
  external_rtc.refresh();
  int year = external_rtc.year() + 2000;
  system_rtc_utc.setTime(external_rtc.second(), external_rtc.minute(), external_rtc.hour(), external_rtc.day(),
                         external_rtc.month(), year);
  Serial.println("Syncing external RTC to system rtc");
}

void processSyncEvent(NTPEvent_t ntpEvent) {
  Serial.printf("[NTP-event] %s\n", NTP.ntpEvent2str(ntpEvent));

  switch (ntpEvent.event) {
    case timeSyncd:
      ntp_is_synced = 1;
      updateExternalRtcTime();
      break;
    case requestSent:
      break;
    case partlySync:
      break;
    case accuracyError:
      break;
    case syncNotNeeded:
      ntp_is_synced = 1;
      updateExternalRtcTime();
      break;
    default:
      if (rtc_failure) {
        return;
      }
      if (last_ntp_error_timestamp + 300 < getEpoch()) {
        // only run this once every 5 min
        syncExtRtcToSystemRtc();
        char buf[255];
        strcat(buf, "NTP Error: ");
        strcat(buf, NTP.ntpEvent2str(ntpEvent));
        displayError(buf);
        last_ntp_error_timestamp = getEpoch();
      }
  }
}


void MqttConnect() {
  Serial.printf("\nConnecting to mqtt host %s:%s...", mqtt_server, mqtt_port);

  int connectTimeout = 10;  // n seconds to wait * 2
  while (!mqtt_ciient.connect(DEFAULT_DEVICE_NAME) && connectTimeout > 0) {
    Serial.print(".");
    connectTimeout--;
    delay(500);
  }
  if (mqtt_ciient.connected()) {
    mqtt_ciient.subscribe(mqtt_queue);
    Serial.println(" MQTT Connected!");
    last_mqtt_connect_attempt = 0;
  } else {
    Serial.println(" MQTT connection failed");
    displayError("MQTT Error: could not connect to broker");
  }
}

void saveConfig() {
  // save the config to flash
  Serial.println("saving config");
#if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
  JsonDocument json;
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif

  json["hostname"] = hostname;
  json["mqtt_enabled"] = mqtt_enabled;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_queue"] = mqtt_queue;
  json["ntp_pool"] = ntp_pool;
  json["tz_offset"] = tz_offset;
  json["dst_active"] = dst_active;
  json["display_color_day"] = display_color_day;
  json["display_color_night"] = display_color_night;
  json["display_color_error"] = display_color_error;
  json["night_mode_start"] = night_mode_start;
  json["night_mode_end"] = night_mode_end;

  File configFile = FFat.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

#if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
  serializeJson(json, Serial);
  serializeJson(json, configFile);
#else
  json.printTo(Serial);
  json.printTo(configFile);
#endif
  configFile.close();
}

// callback for when config is changed in webui and needs to be saved
void saveConfigCallback() {
  strcpy(hostname, custom_hostname.getValue());
  mqtt_enabled = (strncmp(custom_mqtt_enabled_checkbox.getValue(), "true", 1) == 0);
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_queue, custom_mqtt_queue.getValue());
  strcpy(ntp_pool, custom_ntp_pool.getValue());
  strcpy(tz_offset, custom_tz_offset.getValue());
  dst_active = (strncmp(custom_dst_active_checkbox.getValue(), "true", 1) == 0);
  strcpy(display_color_day, custom_display_color_day.getValue());
  strcpy(display_color_night, custom_display_color_night.getValue());
  strcpy(display_color_error, custom_display_color_error.getValue());
  strcpy(night_mode_start, custom_night_mode_start.getValue());
  strcpy(night_mode_end, custom_night_mode_end.getValue());

  saveConfig();

  delay(500);
  Serial.println("Restartig after config update");
  reboot_required = true;
}


void readConfig() {
  // read configuration file from FS
  Serial.println("mounting FS...");

  if (FFat.begin()) {
    Serial.println("mounted file system");
    if (FFat.exists("/config.json")) {
      // file exists, reading and loading
      Serial.println("Config file exists");
      File configFile = FFat.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        config_file_exists = 1;
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        JsonDocument json;
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);

        if (!deserializeError) {
          if (!json["hostname"].isNull()) {
            strcpy(hostname, json["hostname"]);
          }
          if (!json["mqtt_enabled"].isNull()) {
            mqtt_enabled = json["mqtt_enabled"];
          }
          if (!json["mqtt_server"].isNull()) {
            strcpy(mqtt_server, json["mqtt_server"]);
          }
          if (!json["mqtt_port"].isNull()) {
            strcpy(mqtt_port, json["mqtt_port"]);
          }
          if (!json["mqtt_queue"].isNull()) {
            strcpy(mqtt_queue, json["mqtt_queue"]);
          }
          if (!json["ntp_pool"].isNull()) {
            strcpy(ntp_pool, json["ntp_pool"]);
          }
          if (!json["tz_offset"].isNull()) {
            strcpy(tz_offset, json["tz_offset"]);
          }
          if (!json["dst_active"].isNull()) {
            dst_active = json["dst_active"];
          }
          if (!json["display_color_day"].isNull()) {
            strcpy(display_color_day, json["display_color_day"]);
          }
          if (!json["display_color_night"].isNull()) {
            strcpy(display_color_night, json["display_color_night"]);
          }
          if (!json["display_color_error"].isNull()) {
            strcpy(display_color_error, json["display_color_error"]);
          }
          if (!json["night_mode_start"].isNull()) {
            strcpy(night_mode_start, json["night_mode_start"]);
          }
          if (!json["night_mode_end"].isNull()) {
            strcpy(night_mode_end, json["night_mode_end"]);
          }

          Serial.println("\nparsed json");
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }

  } else {
    Serial.println("failed to mount FS");
  }
}

/********************************************************************/

void setup() {
  Serial.begin(115200);
  delay(1500);  // wait for usb device to register for debugging over serial
  Serial.println("Booting");

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  neo_pixel.begin();
  setNeoPixelColor(150, 150, 0);  // yellow means booting

  // get mac addr for device name
  uint8_t mac[8];
  esp_efuse_mac_get_default(mac);
  char tmpBuffer[3];
  sprintf(tmpBuffer, "%02X", mac[4]);
  strcat(DEFAULT_DEVICE_NAME, tmpBuffer);
  sprintf(tmpBuffer, "%02X", mac[5]);
  strcat(DEFAULT_DEVICE_NAME, tmpBuffer);

  readConfig();

  if (*hostname == 0) {
    strcat(hostname, DEFAULT_DEVICE_NAME);
  }

  Serial.printf("%s v%s\n", PROG_NAME, VERSION);
  Serial.printf("Hostname: %s\n", hostname);
  Serial.printf("Mac address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  WiFi.mode(WIFI_STA);

  URTCLIB_WIRE.begin();
  // ensure rtc is working. if not, just display error forever
  external_rtc.refresh();
  if (external_rtc.temp() == 9999) {
    rtc_failure = 1;
    setNeoPixelColor(150, 0, 0);
    displayError("RTC Error: RTC is returning invalid data or not detected", LONG_LONG_MAX);
  }

  // update sys time to try to speed up ntp sync. also do this here to prevent issues
  // with clock change causing errors to not display
  syncExtRtcToSystemRtc();

  display_color_error_hex = StrToHex(display_color_error);
  display_color_day_hex = StrToHex(display_color_day);
  display_color_night_hex = StrToHex(display_color_night);

  // update localized clock offset now that we have the tz info
  float offsetInSeconds = (std::stof(tz_offset) * 3600) + (int(dst_active) * 3600);
  system_rtc_localtime.offset = offsetInSeconds;

  // update custom param elements
  custom_hostname.setValue(hostname, 128);
  custom_mqtt_enabled_checkbox.setValue((mqtt_enabled ? "true" : "false"), 5);
  custom_mqtt_server.setValue(mqtt_server, 128);
  custom_mqtt_port.setValue(mqtt_port, 5);
  custom_mqtt_queue.setValue(mqtt_queue, 128);

  custom_ntp_pool.setValue(ntp_pool, 128);
  custom_tz_offset.setValue(tz_offset, 5);
  custom_dst_active_checkbox.setValue((dst_active ? "true" : "false"), 5);

  custom_display_color_day.setValue(display_color_day, 7);
  custom_display_color_night.setValue(display_color_night, 7);
  custom_display_color_error.setValue(display_color_error, 7);

  custom_night_mode_start.setValue(night_mode_start, 2);
  custom_night_mode_end.setValue(night_mode_end, 2);

  // set config save notify callback
  wifi_manager_inst.setSaveParamsCallback(saveConfigCallback);
  // update checkbox state depending on value as the html can't be modified after init
  wifi_manager_inst.setCustomHeadElement(
    "<script>"
    "document.addEventListener('DOMContentLoaded', function() {"
    "  const checkboxes = document.querySelectorAll('input[type=\"checkbox\"]');"
    "  checkboxes.forEach(checkbox => {"
    "    checkbox.checked = checkbox.value === 'true';"
    "  });"
    "});"
    "</script>"
    "<script>"
    "document.addEventListener('DOMContentLoaded', function() {"
    "  const checkboxes = document.querySelectorAll('input[type=\"checkbox\"]');"
    "  checkboxes.forEach(checkbox => {"
    "    checkbox.addEventListener('change', function() {"
    "      if (this.checked) {"
    "        this.value = 'true';"
    "      } else {"
    "        this.value = 'false';"
    "      }"
    "    });"
    "  });"
    "});"
    "</script>");

  // TODO get this working to show params page
  const char* menuhtml = "<form action='/custom' method='get'><button>Custom</button></form><br/>\n";
  // wifi_manager_inst.setCustomMenuHTML("<form action='/logger' method='get'><button>Log</button></form><br/>\n");
  //  wifi_manager_inst.setCustomMenuHTML("<h2>Device Information</h2><p>This device monitors environmental conditions such as temperature and humidity. If certain sensors detect smoke, water leakage, or loss of AC power, an alarm will be triggered.</p>");
  wifi_manager_inst.setCustomMenuHTML(menuhtml);

  wifi_manager_inst.addParameter(&custom_hostname);
  wifi_manager_inst.addParameter(&custom_ntp_pool);
  wifi_manager_inst.addParameter(&custom_tz_offset);
  wifi_manager_inst.addParameter(&custom_dst_active_checkbox);
  wifi_manager_inst.addParameter(&custom_linebreak);

  wifi_manager_inst.addParameter(&custom_mqtt_enabled_checkbox);
  wifi_manager_inst.addParameter(&custom_linebreak);
  wifi_manager_inst.addParameter(&custom_mqtt_server);
  wifi_manager_inst.addParameter(&custom_mqtt_port);
  wifi_manager_inst.addParameter(&custom_mqtt_queue);

  wifi_manager_inst.addParameter(&custom_linebreak);
  wifi_manager_inst.addParameter(&custom_linebreak);
  wifi_manager_inst.addParameter(&custom_display_color_info);
  wifi_manager_inst.addParameter(&custom_display_color_day);
  wifi_manager_inst.addParameter(&custom_display_color_night);
  wifi_manager_inst.addParameter(&custom_display_color_error);
  wifi_manager_inst.addParameter(&custom_night_mode_start);
  wifi_manager_inst.addParameter(&custom_night_mode_end);

  // don't block so that the clock can start if wifi fails
  wifi_manager_inst.setConfigPortalBlocking(false);
  wifi_manager_inst.setConfigPortalTimeout(120);

  wifi_manager_inst.startWebPortal();
  wifi_manager_inst.setHostname(hostname);

  // automatically connect using saved credentials if they exist
  // If connection fails it starts an access point with the specified name
  if (wifi_manager_inst.autoConnect(hostname, AP_PASSWORD)) {
    Serial.println("Wifi connected");
    wifi_first_connection = true;
  } else {
    Serial.println("Configportal running");
    if (!wifi_manager_inst.getWiFiSSID()) {
      setNeoPixelColor(150, 0, 0);
      char err_msg[255] = "Wifi not configured. Connect to ";
      strcat(err_msg, PROG_NAME);
      strcat(err_msg, " to configure wifi settings");
      displayError(err_msg, 20);
    } else {
      setNeoPixelColor(150, 0, 0);
      displayError("Wifi cannot connect", 20);
    }
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {
        // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

#ifdef ENABLE_OTA_UPDATES
  Serial.println("Enabling OTA updates");
  ArduinoOTA.begin();
#endif

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Wire.begin();

  ProtomatterStatus status = matrix.begin();
  Serial.printf("Protomatter begin() status: %d\n", status);

  // TODO handle this, but I'm unsure when/how this would happen as when
  //   the display is not connected there is no error
  if (status != PROTOMATTER_OK) {
    // DO NOT CONTINUE if matrix setup encountered an error.
    setNeoPixelColor(150, 0, 0);
    Serial.printf("PROTOMATER FAILURE!!!");
    for (;;)
      ;
  }

  matrix.setFont(BOOTUP_MSG_FONT);
  matrix.setCursor(0, CLOCK_LAYOUT_DOW_VERT);

  matrix.println(centerText("Matrix"));
  matrix.println(centerText("Clock"));
  matrix.println(centerText(VERSION));
  matrix.show();  // Copy data to matrix buffers
  delay(1000);

  NTP.onNTPSyncEvent([](NTPEvent_t event) {
    ntpEvent = event;
    sync_event_triggered = true;
  });

  if (mqtt_server[0] != 0 && mqtt_port[0] != 0 && mqtt_queue[0] != 0) {
    mqtt_ciient.setHost(mqtt_server, atoi(mqtt_port));
    mqtt_ciient.setTimeout(10);
    mqtt_ciient.begin(wifi_client_con);
    mqtt_ciient.onMessage(mqttMessageReceived);
  } else {
    setNeoPixelColor(150, 0, 0);
    Serial.println("MQTT cannot be enabled. Server, port, and queue must be set");
  }

  night_mode_start_int = atoi(night_mode_start);
  night_mode_end_int = atoi(night_mode_end);
}

void prepScrollingText(char* text) {
  // prep global vars for msg position
  char scrolling_str[512];  // Buffer to hold scrolling message text
  sprintf(scrolling_str, text, matrix.width(), matrix.height());
  matrix.setTextWrap(false);  // Allow text off edge
  int16_t x1, y1;
  uint16_t w, h;
  matrix.getTextBounds(scrolling_str, 0, 0, &x1, &y1, &w, &h);  // How big is it?
  text_pos_min = -w;                                            // All text is off left edge when it reaches this point
  text_pos_x = matrix.width();                                  // Start off right edge
}

/********************************************************************/
void loop() {
  wifi_manager_inst.process();
  ArduinoOTA.handle();
  external_rtc.refresh();

  matrix.fillScreen(0);  // Fill background black
  bool displayingError = false;

  int BUTTON_UPValue = digitalRead(BUTTON_UP);
  int BUTTON_DOWNValue = digitalRead(BUTTON_DOWN);

  if (BUTTON_UPValue == LOW) {
    if (!dst_active) {
      dst_active = true;
      delay(500);
      saveConfig();
      reboot_required = true;
    }
  } else if (BUTTON_DOWNValue == LOW) {
    if (dst_active) {
      dst_active = false;
      delay(500);
      saveConfig();
      reboot_required = true;
    }
  }

  // things that depend on wifi / first itteration of this loop after wifi connects
  if (wifi_first_connection) {
    wifi_first_connection = false;
    NTP.setInterval(NTP_UPDATE_INTERVAL);
    NTP.setNTPTimeout(NTP_TIMEOUT);
    // NTP.setMinSyncAccuracy (5000);
    // NTP.settimeSyncThreshold (3000);

    char* activeNtpPool;
    String net_gateway = IpAddress2String(WiFi.gatewayIP());

    if (ntp_pool[0] != 0) {
      activeNtpPool = ntp_pool;
    } else {
      Serial.println("NTP pool not given, using default gateway as ntp peer");
      activeNtpPool = &net_gateway[0];
    }

    Serial.println("Starting ntp client");
    NTP.begin(activeNtpPool);
    setNeoPixelColor(0, 0, 0);
  }

  // if there is no network connection then don't attempt mqtt connection or things will hang
  // only attempt to connect every 300s as it is blocking call
  if (WiFi.isConnected() && mqtt_enabled && mqtt_server[0] != 0 && mqtt_port[0] != 0 && mqtt_queue[0] != 0 && last_mqtt_connect_attempt + 300 < getEpoch()) {
    mqtt_ciient.loop();

    // only attempt to connect every so often as it is blocking
    if (!mqtt_ciient.connected()) {
      last_mqtt_connect_attempt = getEpoch();
      MqttConnect();
    }
  }

  if (sync_event_triggered) {
    sync_event_triggered = false;
    processSyncEvent(ntpEvent);
  }

  // skip requring initital ntp sync if systemrtc seems to be set correctly (greater than year '00)
  if (!ntp_is_synced && !last_ntp_error_timestamp && system_rtc_localtime.getYear() == 0) {
    matrix.fillScreen(0);  // Fill background black
    matrix.setCursor(1, 2);
    matrix.println(centerText("Waiting"));
    matrix.println(centerText("for NTP"));
    matrix.println(centerText("sync"));
    matrix.show();
    // if a min has passed, give up and use rtc
    if (millis() > 60000) {
      Serial.println("Timeout waiting for NTP sync. Using external RTC and will continue ntp sync in background.");
      syncExtRtcToSystemRtc();
      displayError("NTP Error: NTP server reachable but timedout waiting for NTP sync. Will retry in background...");
    }
  } else if (message_being_displayed || nextIndexInMessageQueue() > -1) {
    // message being displayed or there is one to display

    if (!message_being_displayed) {
      internal_message_queue_current_index = nextIndexInMessageQueue();
      current_message = strdup(internal_message_queue[internal_message_queue_current_index]["message"]);
      current_message_repeats = internal_message_queue[internal_message_queue_current_index]["repeat"];
      if (current_message_repeats < 1) {
        // default to 3 repeats if not specified (or something invalid was given)
        current_message_repeats = 3;
      }
      if (internal_message_queue[internal_message_queue_current_index]["color"].isNull()) {
        current_message_color_hex = 0;
      } else {
        const char* current_messageColor = internal_message_queue[internal_message_queue_current_index]["color"].as<const char*>();
        current_message_color_hex = StrToHex(const_cast<char*>(current_messageColor));
      }
      message_being_displayed = true;
      prepScrollingText(current_message);
    }

    // there is a msg and is msg an error?
    if (internal_message_queue[internal_message_queue_current_index]["isError"]) {
      matrix.setTextColor(display_color_error_hex);

      // Draw the scrolling text
      matrix.setCursor(text_pos_x, ERROR_POSITION_VERT);
      matrix.print(current_message);

      // Update text position for next frame. If text goes off the
      // left edge, reset its position to be off the right edge.
      if ((--text_pos_x) < text_pos_min) {
        text_pos_x = matrix.width();
        current_message_repeats--;
      }
      if (current_message_repeats == 0) {
        message_being_displayed = false;
      }

      matrix.show();
      delay(20);
      displayingError = true;
    } else {
      // there is a msg and it is not an error, display instead of dow
      if (current_message_color_hex > 0) {
        matrix.setTextColor(current_message_color_hex);
      } else {
        if (system_rtc_localtime.getHour(true) >= night_mode_start_int || system_rtc_localtime.getHour(true) < night_mode_end_int) {
          matrix.setTextColor(display_color_night_hex);
        } else {
          matrix.setTextColor(display_color_day_hex);
        }
      }
      // display scrolling text
      matrix.setCursor(text_pos_x, CLOCK_LAYOUT_DOW_VERT);
      matrix.setFont(CLOCK_LAYOUT_DOW_FONT);
      matrix.print(current_message);

      // Update text position for next frame. If text goes off the
      // left edge, reset its position to be off the right edge.
      if ((--text_pos_x) < text_pos_min) {
        text_pos_x = matrix.width();
        current_message_repeats--;
      }
      if (current_message_repeats == 0) {
        message_being_displayed = false;
      }
      delay(20);
    }
  } else {
    // no msgs, display dow
    matrix.setFont(CLOCK_LAYOUT_DOW_FONT);
    matrix.setCursor(CLOCK_LAYOUT_DOW_HORIZ, CLOCK_LAYOUT_DOW_VERT);
    matrix.println(centerText(days_of_week[system_rtc_localtime.getDayofWeek()]));
  }

  if (!displayingError) {
    // display time/date if no error to display
    if (system_rtc_localtime.getHour(true) >= night_mode_start_int || system_rtc_localtime.getHour(true) < night_mode_end_int) {
      matrix.setTextColor(display_color_night_hex);
    } else {
      matrix.setTextColor(display_color_day_hex);
    }

    // display date and time info
    matrix.setFont(CLOCK_LAYOUT_TIME_FONT);
    matrix.setCursor(CLOCK_LAYOUT_TIME_HORIZ, CLOCK_LAYOUT_TIME_VERT);
    matrix.print(padNumber(system_rtc_localtime.getHour(true)));
    matrix.print(":");
    matrix.print(padNumber(system_rtc_localtime.getMinute()));
    matrix.print(":");
    matrix.println(padNumber(system_rtc_localtime.getSecond()));

    matrix.setFont(CLOCK_LAYOUT_DATE_FONT);
    matrix.setCursor(CLOCK_LAYOUT_DATE_HORIZ, CLOCK_LAYOUT_DATE_VERT);
    matrix.print(padNumber(system_rtc_localtime.getDay()));
    matrix.print("/");
    matrix.print(padNumber(system_rtc_localtime.getMonth() + 1));
    // rtc only returns two digit year
    matrix.print("/");
    matrix.println(system_rtc_localtime.getYear());

    matrix.show();  // Copy data to matrix buffers (display it)
  }

  if (reboot_required) {
    ESP.restart();
  }
}
