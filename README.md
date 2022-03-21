# ESP32C3_BLUETOOTH_AUDIO_VARIO
 Accurate, zero-lag audio variometer using Kalman filter fusion of accelerometer and pressure sensor data. 
 This project uses the [KF4D kalman filter algorithm from the ESP32_IMU_GPS_BARO_VARIO project](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/offline/kf/compare_kf2_kf3_kf4.ipynb).

 Other features :
* WiFi Vario configuration via web page.
* WiFi Over-the-air (OTA) firmware updates. 
* Bluetooth LE transmission of $LK8EX1 sentences. You can use flight instrument Android apps like [XCTrack](https://xctrack.org/) with 
accurate barometric pressure altitude and climb-rate data.
* Soft power on/off button.
* No-activity power-down to save battery life.
* USB-C Li-poly battery charging at up to 500mA.
* PCB sized for standard Hammond enclosure.

# Software Build Environment 
* Ubuntu 20.04 LTS AMDx64
* [Visual Studio Code with PlatformIO plugin using Arduino framework](https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/) 
* The file `platformio.ini` specifies the framework packages, toolchain and libraries used by the project. 
* LittleFS partition for hosting HTML web server pages

# Hardware

* AI-Thinker ESP-C3-12F C3FN4 module (4MByte flash, Wi-Fi and Bluetooth LE)
* CJMCU-117 module with MPU9250 9-DOF IMU and MS5611 barometric pressure sensor
* KPEG006 broadband-audio piezo transducer
* 74HC240 used as a push-pull piezo driver for louder volume 
* MCP73871 Li-poly battery charger, max 500mA charging current
* 1800mAHr Lipoly battery
* Hammond 1551K standard size enclosure (80 x 40 x 20mm). 
* TLV75533 LDO regulator, max current 500mA
* [Kicad schematic and PCB layout](https://github.com/har-in-air/VhARIO-ESPC3)

## Current Drain

With Bluetooth LE disabled, current drain is ~30mA.

With Bluetooth LE enabled and transmitting LK8EX1 messages at 10Hz, current drain is ~85mA.

# Software Build Notes

## Build Steps
* The first time you flash the ESP32-C3 with this project code, select `PROJECT TASKS -> esp32c3 -> Platform -> Erase Flash`. 
* Next, select `Platform -> Build Filesystem Image`. This will build a LittleFS flash partition with the contents of the `/data` directory. The `/data` directory contains the static HTML and CSS files for the WiFi server webpage.
* Next, select `Platform -> Upload Filesystem Image`. This will flash the LittleFS data partition to the ESP32-C3.
* Next, select `General -> Clean All`, then `Build`. You may see (if the issue has not been fixed) this build error : 
```
Compiling .pio/build/esp32c3/libfe7/ESP Async WebServer/WebServer.cpp.o
.pio/libdeps/esp32c3/ESP Async WebServer/src/AsyncWebSocket.cpp: In member function "IPAddress AsyncWebSocketClient::remoteIP()":
.pio/libdeps/esp32c3/ESP Async WebServer/src/AsyncWebSocket.cpp:832:28: error: call of overloaded "IPAddress(unsigned int)" is ambiguous
         return IPAddress(0U);
```
Fix the error by replacing 0U with (uint32_t)0. 
* From now on, only select `General -> Clean` to avoid pulling in the original library source code again.
* Select `Build` and then `Upload and Monitor` to build and flash the application firmware binary.
* Ensure the serial debug monitor is visible, then reset or power-cycle the ESP32-C3 module. Since there is no calibration data, you will see a calibration error message. Follow the prompts to calibrate both accelerometer and gyroscope.
[This is a startup serial monitor log after a full flash erase, i.e. no calibration parameters.](docs/first_boot_log.txt) 
* The gyroscope is re-calibrated each time on power-up. You should leave the vario undisturbed when you hear the count-down beeps for gyroscope calibration. If the vario is disturbed during the gyro calibration process, it will use the last saved gyro calibration parameters.
* [This is a startup serial monitor log of the vario with calibrated accelerometer.](docs/normal_boot_log.txt). 


# WiFi Configuration

To put the vario into WiFi AP server mode, switch on the vario and immediately press and hold the `PCC` button. When you hear a confirmation tone, release PCC. 

Connect to the WiFi Access Point `Vario-AP`, no password needed. 

Open the url `http://192.168.4.1` in a browser.
You can use `http://vario.local` with any OS that has mDNS support. MacOS has built-in support. For Ubuntu, install Avahi. For Windows, install Bonjour.

You can now configure an external WiFi Access Point SSID and password. 
Then you do not have to switch between your home WiFi network and the vario Access Point to be able to configure the vario. 
After configuration, restart the vario and trigger wifi configuration mode again.

Now if the external Access Point is available, the vario will connect to it as a client, and then start the configuration web server. 
If your OS has mDNS support, use the url `http://vario.local` for configuration. 
Else you will have to watch the serial monitor to find the dynamically assigned IP address.

If the configured external AP is not available (or configured with wrong credentials) the vario will fall back to the stand-alone Access Point and web server. 
So you can still configure the vario in the field.

<img src="docs/wifi_config_webpage.png">

# Usage

## Power on and off
To power on, press the PWR button and hold until you see the power LED turn on. Release.

If Bluetooth transmission is enabled, the power LED will start blinking once transmission starts, else it will stay on.

To power off, press the PWR button and hold until you see the LED turn off. Release.

## Audio mute toggle
A brief press of the PCC button while the vario is operational will toggle the audio on and off.
This is convenient if you have set the zeros threshold to a negative value or close to zero and don't want the distraction of a beeping vario while you are on launch.


