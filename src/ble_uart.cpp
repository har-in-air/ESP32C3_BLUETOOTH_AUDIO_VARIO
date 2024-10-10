#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include "config.h"
#include "ble_uart.h"


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID

#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer* pBLEServer                = NULL;
NimBLEService* pService                 = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;

static uint8_t ble_uart_nmea_checksum(const char *szNMEA);
#ifdef AUX_SERIAL
static HardwareSerial auxSerial(portAux);
#endif

void ble_uart_init() {
	NimBLEDevice::init("Bavario");
	NimBLEDevice::setMTU(46);
	// default power level is +3dB, max +9dB
	//NimBLEDevice::setPower(ESP_PWR_LVL_N3); // -3dB
	NimBLEDevice::setPower(ESP_PWR_LVL_N0); // 0dB
    //NimBLEDevice::setPower(ESP_PWR_LVL_P6);  // +6db 

	NimBLEDevice::setSecurityAuth(true, true, true);
	NimBLEDevice::setSecurityPasskey(123456);
	NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);

	pBLEServer = NimBLEDevice::createServer();

	pService          = pBLEServer->createService(SERVICE_UUID);
	pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);

	pRxCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RX,
		NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);

	pService->start();
	pBLEServer->getAdvertising()->start();
#ifdef AUX_SERIAL
	auxSerial.begin(115200, SERIAL_8N1, -1, pinAuxTx);
	while(!auxSerial)
		delay(100);
#endif
}


static uint8_t ble_uart_nmea_checksum(const char *szNMEA){
	const char* sz = &szNMEA[1]; // skip leading '$'
	uint8_t cksum = 0;
	while ((*sz) != 0 && (*sz != '*')) {
		cksum ^= (uint8_t) *sz;
		sz++;
		}
	return cksum;
}
   
void ble_uart_transmit(const char *msg) {
#ifdef BLE_DEBUG	
    dbg_printf(("bleTX: %s", msg)); 
#endif
#ifdef AUX_SERIAL
	auxSerial.write(msg);
#endif
	const int maxPacketSize = 20;
	for(int length = strlen(msg); length > 0; length -= maxPacketSize) {
		pTxCharacteristic->setValue((const uint8_t*)msg, MIN(maxPacketSize, strlen(msg)));
		pTxCharacteristic->notify();   
		msg += maxPacketSize;
	}
}

void ble_uart_transmit_LK8EX1(int32_t altm, int32_t cps, float batPercentage) {
	char szmsg[40];
	sprintf(szmsg, "$LK8EX1,999999,%d,%d,99,%.0f*", altm, cps, 1000.0f + batPercentage);
	uint8_t cksum = ble_uart_nmea_checksum(szmsg);
	char szcksum[5];
	sprintf(szcksum,"%02X\r\n", cksum);
	strcat(szmsg, szcksum);
	ble_uart_transmit(szmsg);
}
