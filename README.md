# ESP32C3_BLUETOOTH_AUDIO_VARIO

* Accurate, zero-lag audio feedback variometer using Kalman filter fusion of accelerometer and pressure sensor data
* WiFi webpage configuration with the vario acting as an access point and web server.
* WiFi Over-the-air (OTA) firmware update 
* Bluetooth transmission of LK8EX1 sentences. You can then use 
flight instrument apps like [XCTrack](https://xctrack.org/) on a smartphone/tablet with accurate barometric altitude and climb/sink rate data.
* You can optionally add these features : 
    * Push-pull L9110S driver with conventional loudspeakers for higher audio volume. 
    * Torch/lantern mode using a 0.5W white led. This is accessed with a long press of a button when the unit is in vario mode. Once in lantern mode, you can cycle through 3 different brightness levels and an S.O.S. flasher mode.

# Software Build Environment 
* Ubuntu 20.04 LTS AMDx64
* Visual Studio Code with PlatformIO plugin using Arduino framework targeting `esp32dev` board. The file `platformio.ini` specifies the framework packages and toolchain required for the ESP32-C3, and the libraries used by the project. ESP32-C3 Arduino framework support is new and not as solid as for the ESP32. A minor type-cast compile error for the AsyncTCP library had to be fixed by editing the local version of the library source code in the project `.pio` subdirectory.
* Custom `partition.csv` file with two 1.9MB code partitions supporting OTA firmware updates
* ~160kByte LittleFS partition for hosting HTML web server pages

# Hardware

* ESP32-C3 12F 4MByte flash (AI Thinker)
* CJMCU-117 10-DOF IMU module with MPU9250 and MS5611 sensors
* L9110S IC used as a push-pull driver for louder volume 

# Hardware Notes

The 10K I2C pullup resistors on the CJMCU-117 board should be replaced with 3.3K for a reliable interface at 400kHz.

The optional circuit components are marked with dashes on the schematic. Do not populate them if 
you don't want the torch option or the L9110s loud(er) speaker option. 

You can first test the board with a direct connection from AUD pin to a piezo speaker and the other piezo
pin connected to ground. 
If you mount the vario on your shoulder and/or have an open-face helmet, the volume level is probably enough without populating the L9110S push-pull driver circuit.

If you want louder audio, install the L9110s circuit option. If this is still not loud enough, replace the piezo transducer with a magnetic coil loudspeaker of at least 8 ohms impedance. Make sure you use at least a 47 ohm resistor for R5 to keep current pulses manageable. 
Ensure there is no air path from the front of the loudspeaker 
to back or else the front wave will cancel the back wave. 
Use silicone  to seal the edge of the speaker to the pcb, that will do the job. 
Put some soft foam tape on the back of the speaker so that vibrations don't get transmitted 
to the MPU9250 or MS5611.

Replacement speaker-phone (NOT earpiece) drivers for mobile phones are a good choice.  You can put two in series for 16ohms impedance, but make sure they are in phase.

A few components may not be readily available on Aliexpress/Ebay. You can find them on Mouser/Digikey :
* Ferrite bead 600ohms@100MHz : BLM18AG601SN1D
* TI TPS22918 high-side switch 
* Broadband piezo speaker : PUI Audio AT2310TLW100R, Kingstate KPEG006 
* Power switch : ALPS SSSS916400 (good quality, expensive) or SK12D07 (ebay, cheap, cut off the end lugs).
* For torch LEDs up to 0.5W, populate one of R1, R2 with a 22ohm 2512 0.5W package. For higher wattage LEDs, add a second resistor in parallel. 


Battery current drain is `~xxmA` operating as audio vario with bluetooth LE disabled. 

Battery current drain is `~xxmA` operating as audio vario with bluetooth LE LK8EX1 message transmission @ 10Hz.

# Software Build Notes

* For a minimal audio vario with the ESP32-C3 directly driving a piezo transducer, edit the file `config.h` and set `CFG_L9110S` and `CFG_LANTERN` to false. If you want support for louder volume using the L9110S push-pull driver IC, set `CFG_L9110S` to true.  
* To support the torch/lantern feature, set `CFG_LANTERN` to true.
* The first time you flash the ESP32-C3 with this project code, select `PROJECT TASKS -> esp32c3 -> Platform -> Erase Flash`. 
* Next, select `Platform -> Build Filesystem Image`. This will build a LittleFS flash partition with the contents of the `/data` directory. The `/data` directory contains the static HTML and CSS files for the WiFi server webpage.
* Next, select `Platform -> Upload Filesystem Image`. This will flash the LittleFS data partition to the ESP32-C3.
* Next, select `General -> Clean`, then `Build` and then `Upload and Monitor` to build and flash the application firmware binary.
* Ensure the serial debug monitor is visible, then reset or power-cycle the ESP32-C3 module. Since there is no calibration data, you will see a calibration error message. Follow the prompts to calibrate both accelerometer and gyroscope.
[This is a startup serial monitor log after a full flash erase.](docs/calibration_log.txt). 
* The gyroscope is re-calibrated each time on power-up. You should leave the vario undisturbed when you hear the count-down beeps for gyroscope calibration. If the vario is disturbed during the gyro calibration process, it will use the last saved gyro calibration parameters.
* [This is a startup serial monitor log of the vario with calibrated accelerometer.](docs/boot_log.txt). 
* This project uses the KF4D kalman filter algorithm from the [ESP32_IMU_GPS_BARO_VARIO](https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO) project.
* To put the vario into WiFi configuration mode, switch on the vario and immediately press the `PCC` button. Keep it pressed until you hear a confirmation tone, then release. You can now connect to the WiFi Access Point `ESPC3-Vario` - no password needed. Now, access the url `http://192.168.4.1` in a browser. If your OS has mDNS support, you can also use the url `http://vario.local`.  MacOS has built-in mDNS support. On Windows, install Bonjour, on Ubuntu Linux install Avahi.
<img src="docs/wifi_config_webpage.png">
* To update the firmware, access the url `http://192.168.4.1/update` or `http://vario.local/update`. 
Upload the new firmware binary `.bin` file. Restart the vario. Select WiFi configuration mode again, and confirm the firmware revision string has changed (assuming it has been updated along with code changes).
<br><br><img src="docs/firmware_update.png">
