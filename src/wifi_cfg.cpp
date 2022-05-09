#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include "config.h"
#include "nvd.h"
#include "adc.h"
#include "wifi_cfg.h"

static const char* TAG = "wificfg";

extern const char* FwRevision;

const char* szAPSSID = "Vario-AP";
const char* szAPPassword = ""; // no password for stand-alone access point

AsyncWebServer* pServer = NULL;



static void wifi_start_as_ap();
static void wifi_start_as_station();

static String server_string_processor(const String& var);
static void not_found_handler(AsyncWebServerRequest *request);
static void get_handler(AsyncWebServerRequest *request);
static void index_page_handler(AsyncWebServerRequest *request);
static void set_defaults_handler(AsyncWebServerRequest *request);
static void css_handler(AsyncWebServerRequest *request);

	
void wifi_off() {
    WiFi.mode(WIFI_MODE_NULL);
    esp_wifi_stop();
    delay(100);
    }
  

// Replace %xx% placeholder 
static String server_string_processor(const String& var){
    if(var == "FIRMWARE_REVISION"){
        return FwRevision;
        }
    else
    if(var == "BATTERY_VOLTAGE"){
        return String(BatteryVoltage, 2);
        }
	else
	if(var == "SSID"){
		return Config.cred.ssid;
		}
	else
	if(var == "PASSWORD"){
		return Config.cred.password;
		}
	else	
    if(var == "TIMEOUT_MIN"){
        return String(PWR_OFF_TIMEOUT_MINUTES_MIN);
        }
    else
    if(var == "TIMEOUT_MAX"){
        return String(PWR_OFF_TIMEOUT_MINUTES_MAX);
        }
    else
    if(var == "TIMEOUT"){
        return String(Config.misc.pwrOffTimeoutMinutes);
        }    
    else
    if(var == "AVAR_MIN"){
        return String(KF_ACCEL_VARIANCE_MIN);
        }
    else
    if(var == "AVAR_MAX"){
        return String(KF_ACCEL_VARIANCE_MAX);
        }
    else
    if(var == "AVAR"){
        return String(Config.kf.accelVariance);
        }
    else
    if(var == "ZVAR_MIN"){
        return String(KF_ZMEAS_VARIANCE_MIN);
        }
    else
    if(var == "ZVAR_MAX"){
        return String(KF_ZMEAS_VARIANCE_MAX);
        }
    else
    if(var == "ZVAR"){
        return String(Config.kf.zMeasVariance);
        }
	else
    if(var == "CLIMB_MIN"){
        return String(VARIO_CLIMB_THRESHOLD_CPS_MIN);
        }
    else
    if(var == "CLIMB_MAX"){
        return String(VARIO_CLIMB_THRESHOLD_CPS_MAX);
        }
    else
    if(var == "CLIMB"){
        return String(Config.vario.climbThresholdCps);
        }
    else
    if(var == "ZERO_MIN"){
        return String(VARIO_ZERO_THRESHOLD_CPS_MIN);
        }
    else
    if(var == "ZERO_MAX"){
        return String(VARIO_ZERO_THRESHOLD_CPS_MAX);
        }
    else
    if(var == "ZERO"){
        return String(Config.vario.zeroThresholdCps);
        }
    else
    if(var == "SINK_MIN"){
        return String(VARIO_SINK_THRESHOLD_CPS_MIN);
        }
    else
    if(var == "SINK_MAX"){
        return String(VARIO_SINK_THRESHOLD_CPS_MAX);
        }
    else
    if(var == "SINK"){
        return String(Config.vario.sinkThresholdCps);
        }
    else
    if(var == "XOVER_MIN"){
        return String(VARIO_CROSSOVER_CPS_MIN);
        }
    else
    if(var == "XOVER_MAX"){
        return String(VARIO_CROSSOVER_CPS_MAX);
        }
    else
    if(var == "XOVER"){
        return String(Config.vario.crossoverCps);
        }
	else 
	if (var == "BLE_OFF") {
		return Config.misc.bleEnable ? "" : "checked";
		}
	else 
	if (var == "BLE_ON") {
		return Config.misc.bleEnable ? "checked" : "";
		}
    else return "?";
    }


static void wifi_start_as_ap() {
	Serial.printf("Starting Access Point %s with password %s\n", szAPSSID, szAPPassword);
	WiFi.softAP(szAPSSID, szAPPassword);
	IPAddress IP = WiFi.softAPIP();
	Serial.print("AP IP address : ");
	Serial.println(IP);
	}


static void wifi_start_as_station() {
	Serial.printf("Connecting as station to SSID %s\n", Config.cred.ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(Config.cred.ssid.c_str(), Config.cred.password.c_str());
    if (WiFi.waitForConnectResult(10000UL) != WL_CONNECTED) {
    	Serial.printf("Connection failed!\n");
    	wifi_start_as_ap();
    	}
	else {
    	Serial.println();
    	Serial.print("Local IP Address: ");
    	Serial.println(WiFi.localIP());
		}
	}

void wificfg_ap_server_init() {
    esp_wifi_start();  
    delay(100);
	if (Config.cred.ssid == "") {
		wifi_start_as_ap();
		}
	else {
		wifi_start_as_station();
		}	
	if (!MDNS.begin("vario")) { // Use http://vario.local for web server page
		Serial.println("Error starting mDNS service");
	    }
    pServer = new AsyncWebServer(80);
    if (pServer == NULL) {
        dbg_println(("Error creating AsyncWebServer!"));
        while(1);
        }

    pServer->onNotFound(not_found_handler);
    pServer->on("/", HTTP_GET, index_page_handler);
    pServer->on("/defaults", HTTP_GET, set_defaults_handler);
    pServer->on("/get", HTTP_GET, get_handler);	
    pServer->on("/style.css", HTTP_GET, css_handler);	

    // add support for OTA firmware update
    AsyncElegantOTA.begin(pServer);
    pServer->begin();
	MDNS.addService("http", "tcp", 80);
    }

static void css_handler(AsyncWebServerRequest *request){
	request->send(LittleFS, "/style.css", "text/css");
	}

static void index_page_handler(AsyncWebServerRequest *request){
	request->send(LittleFS, "/index.html", String(), false, server_string_processor);
	}


static void set_defaults_handler(AsyncWebServerRequest *request) {
	nvd_config_reset(Config);
	request->send(200, "text/html", "Default options set<br><a href=\"/\">Return to Home Page</a>");  
	}


static void get_handler(AsyncWebServerRequest *request) {
	String inputMessage;
	bool bChange = false;
	if (request->hasParam("ssid")) {
		inputMessage = request->getParam("ssid")->value();
		bChange = true; 
		Config.cred.ssid = inputMessage;
		}
	if (request->hasParam("password")) {
		inputMessage = request->getParam("password")->value();
		bChange = true; 
		Config.cred.password = inputMessage; 
		}
	if (request->hasParam("ble")) {
		inputMessage = request->getParam("ble")->value();
		bChange = true; 
		Config.misc.bleEnable = (inputMessage == "ble_on"); 
		}
	if (request->hasParam("climb")) {
		inputMessage = request->getParam("climb")->value();
		bChange = true; 
		Config.vario.climbThresholdCps = inputMessage.toInt();
		}
	if (request->hasParam("sink")) {
		inputMessage = request->getParam("sink")->value();
		bChange = true; 
		Config.vario.sinkThresholdCps = inputMessage.toInt();
		}
	if (request->hasParam("zero")) {
		inputMessage = request->getParam("zero")->value();
		bChange = true; 
		Config.vario.zeroThresholdCps = inputMessage.toInt();
		}
	if (request->hasParam("xover")) {
		inputMessage = request->getParam("xover")->value();
		bChange = true; 
		Config.vario.crossoverCps = inputMessage.toInt();
		}
	if (request->hasParam("avar")) {
		inputMessage = request->getParam("avar")->value();
		bChange = true; 
		Config.kf.accelVariance = inputMessage.toInt();
		}
	if (request->hasParam("zvar")) {
		inputMessage = request->getParam("zvar")->value();
		bChange = true; 
		Config.kf.zMeasVariance = inputMessage.toInt();
		}
	if (request->hasParam("timeout")) {
		inputMessage = request->getParam("timeout")->value();
		bChange = true; 
		Config.misc.pwrOffTimeoutMinutes = inputMessage.toInt();
		}

	if (bChange == true) {
		dbg_println(("Config parameters changed"));
		nvd_config_store(Config);
		bChange = false;
		}
	request->send(200, "text/html", "Input Processed<br><a href=\"/\">Return to Home Page</a>");  
	}


static void not_found_handler(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
    }
