//////------------------------------------
////// Inludes

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "MedianFilter.h"

//////------------------------------------
////// Defines

#define DEBUG_BLE_SCAN 0
#define DEBUG_BLE_NOTIFY 0
#define DEBUG_DISPLAY_FRAME_LCD_TO_CNTRL 0
#define DEBUG_DISPLAY_FRAME_CNTRL_TO_LCD 0
#define DEBUG_DISPLAY_SPEED 1
#define DEBUG_DISPLAY_MODE 0
#define DEBUG_SERIAL_CHECKSUM_LCD_TO_CNTRL 0
#define DEBUG_SERIAL_CHECKSUM_CNTRL_TO_LCD 0

#define PIN_SERIAL_LCD_TO_CNTRL_RX 25
#define SERIAL_LCD_TO_CNTRL_TXPIN 26

#define SERIAL_CNTRL_TO_LCD_RXPIN 34
#define SERIAL_CNTRL_TO_LCD_TXPIN 13

#define PIN_IN_BREAK 32
#define PIN_IN_VOLTAGE 33
#define PIN_IN_CURRENT 35

#define MODE_LCD_TO_CNTRL 0
#define MODE_CNTRL_TO_LCD 1

#define DATA_BUFFER_SIZE 30
#define BAUD_RATE 1200

#define ANALOG_TO_VOLTS 43.48
#define ANALOG_TO_CURRENT 35
#define ANALOG_TO_CURRENT_MOYENNE_ZERO 1911

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SPEED_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a0"
#define MODE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a1"
#define BRAKE_STATUS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a2"
#define VOLTAGE_STATUS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a3"
#define CURRENT_STATUS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a4"
#define POWER_STATUS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a5"
#define BTLOCK_STATUS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a6"

//////------------------------------------
////// Variables

// Time
uint32_t timeLastNotifyBle = 0;
uint32_t timeLastBrake = 0;
uint32_t timeLoop = 0;

int begin_soft = 0;
int begin_hard = 0;

char data_buffer_lcd[DATA_BUFFER_SIZE];
char data_buffer_cntrl[DATA_BUFFER_SIZE];
char data_speed_buffer[4];

HardwareSerial hwSerCntrlToLcd(1);
HardwareSerial hwSerLcdToCntrl(2);

int i_loop = 0;

bool bleLock = false;
bool blePicclyVisible = true;

int i_LcdToCntrl = 0;
int i_CntrlToLcd = 0;
int begin_LcdToCntrl = 1;
int begin_CntrlToLcd = 1;

int isModified_LcdToCntrl = 0;
int isModified_CntrlToLcd = 0;

uint8_t speedCurrent = 0;
uint8_t speedOld = 0;
uint8_t fakeSpeed = 25;

uint8_t modeOrder = 2;

uint8_t brakeStatus = 0;
uint8_t brakeStatusOld = 0;
uint8_t breakeMin = 2;
uint8_t breakeMax = 4;
uint8_t breakeSentOrder = breakeMin;

uint16_t voltageStatus = 0;
uint32_t voltageInMilliVolts = 0;
MedianFilter voltageFilter(100, 0);
MedianFilter currentFilter(100, 0);

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicSpeed = NULL;
BLECharacteristic *pCharacteristicMode = NULL;
BLECharacteristic *pCharacteristicBrakeSentOrder = NULL;
BLECharacteristic *pCharacteristicVoltageStatus = NULL;
BLECharacteristic *pCharacteristicCurrentStatus = NULL;
BLECharacteristic *pCharacteristicPowerStatus = NULL;
BLECharacteristic *pCharacteristicBtlockStatus = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEScan *pBLEScan;

// BYTE 3 FROM LCD TO CONTRL ... for each sequence number // brute force
const byte lcdMode0[256] = {0x80, 0x05, 0x06, 0x2b, 0x34, 0x29, 0x2a, 0x2f, 0x28, 0x2d, 0x2e, 0x53, 0x7c, 0x51, 0x52, 0x57, 0x50, 0x55, 0x56, 0x7b, 0x84, 0x79, 0x7a, 0x7f, 0x78,
                            0x7d, 0x7e, 0x63, 0x0c, 0x61, 0x62, 0x67, 0x60, 0x65, 0x66, 0x0b, 0x14, 0x09, 0x0a, 0x0f, 0x08, 0x0d, 0x0e, 0x33, 0x5c, 0x31, 0x32, 0x37, 0x30, 0x35, 0x36, 0x5b, 0x64, 0x59, 0x5a,
                            0x5f, 0x58, 0x5d, 0x5e, 0x43, 0x6c, 0x41, 0x42, 0x47, 0x40, 0x45, 0x46, 0x6b, 0x74, 0x69, 0x6a, 0x6f, 0x68, 0x6d, 0x6e, 0x13, 0x3c, 0x11, 0x12, 0x17, 0x10, 0x15, 0x16, 0x3b,
                            0x44, 0x39, 0x3a, 0x3f, 0x38, 0x3d, 0x3e, 0x23, 0x4c, 0x21, 0x22, 0x27, 0x20, 0x25, 0x26, 0x4b, 0x54, 0x49, 0x4a, 0x4f, 0x48, 0x4d, 0x4e, 0x73, 0x1c, 0x71, 0x72, 0x77, 0x70,
                            0x75, 0x76, 0x1b, 0x24, 0x19, 0x1a, 0x1f, 0x18, 0x1d, 0x1e, 0x83, 0x2c, 0x81, 0x82, 0x07, 0x80, 0x05, 0x06, 0x2b, 0x34, 0x29, 0x2a, 0x2f, 0x28, 0x2d, 0x2e, 0x53, 0x7c, 0x51, 0x52,
                            0x57, 0x50, 0x55, 0x56, 0x7b, 0x84, 0x79, 0x7a, 0x7f, 0x78, 0x7d, 0x7e, 0x63, 0x0c, 0x61, 0x62, 0x67, 0x60, 0x65, 0x66, 0x0b, 0x14, 0x09, 0x0a, 0x0f, 0x08, 0x0d, 0x0e, 0x33, 0x5c,
                            0x31, 0x32, 0x37, 0x30, 0x35, 0x36, 0x5b, 0x64, 0x59, 0x5a, 0x5f, 0x58, 0x5d, 0x5e, 0x43, 0x6c, 0x41, 0x42, 0x47, 0x40, 0x45, 0x46, 0x6b, 0x74, 0x69, 0x6a, 0x6f, 0x68, 0x6d,
                            0x6e, 0x13, 0x3c, 0x11, 0x12, 0x17, 0x10, 0x15, 0x16, 0x3b, 0x44, 0x39, 0x3a, 0x3f, 0x38, 0x3d, 0x3e, 0x23, 0x4c, 0x21, 0x22, 0x27, 0x20, 0x25, 0x26, 0x4b, 0x54, 0x49, 0x4a,
                            0x4f, 0x48, 0x4d, 0x4e, 0x73, 0x1c, 0x71, 0x72, 0x77, 0x70, 0x75, 0x76, 0x1b, 0x24, 0x19, 0x1a, 0x1f, 0x18, 0x1d, 0x1e, 0x83, 0x2c, 0x81, 0x82, 0x7};
const byte lcdMode1[256] = {0x85, 0x0a, 0x0b, 0x30, 0x39, 0x2e, 0x2f, 0x34, 0x2d, 0x32, 0x33, 0x58, 0x81, 0x56, 0x57, 0x5c, 0x55, 0x5a, 0x5b, 0x80, 0x89, 0x7e, 0x7f, 0x84,
                            0x7d, 0x82, 0x83, 0x68, 0x11, 0x66, 0x67, 0x6c, 0x65, 0x6a, 0x6b, 0x10, 0x19, 0x0e, 0x0f, 0x14, 0x0d, 0x12, 0x13, 0x38, 0x61, 0x36, 0x37, 0x3c, 0x35, 0x3a, 0x3b, 0x60, 0x69,
                            0x5e, 0x5f, 0x64, 0x5d, 0x62, 0x63, 0x48, 0x71, 0x46, 0x47, 0x4c, 0x45, 0x4a, 0x4b, 0x70, 0x79, 0x6e, 0x6f, 0x74, 0x6d, 0x72, 0x73, 0x18, 0x41, 0x16, 0x17, 0x1c, 0x15, 0x1a,
                            0x1b, 0x40, 0x49, 0x3e, 0x3f, 0x44, 0x3d, 0x42, 0x43, 0x28, 0x51, 0x26, 0x27, 0x2c, 0x25, 0x2a, 0x2b, 0x50, 0x59, 0x4e, 0x4f, 0x54, 0x4d, 0x52, 0x53, 0x78, 0x21, 0x76, 0x77,
                            0x7c, 0x75, 0x7a, 0x7b, 0x20, 0x29, 0x1e, 0x1f, 0x24, 0x1d, 0x22, 0x23, 0x88, 0x31, 0x86, 0x87, 0x0c, 0x85, 0x0a, 0x0b, 0x30, 0x39, 0x2e, 0x2f, 0x34, 0x2d, 0x32, 0x33, 0x58,
                            0x81, 0x56, 0x57, 0x5c, 0x55, 0x5a, 0x5b, 0x80, 0x89, 0x7e, 0x7f, 0x84, 0x7d, 0x82, 0x83, 0x68, 0x11, 0x66, 0x67, 0x6c, 0x65, 0x6a, 0x6b, 0x10, 0x19, 0x0e, 0x0f, 0x14, 0x0d,
                            0x12, 0x13, 0x38, 0x61, 0x36, 0x37, 0x3c, 0x35, 0x3a, 0x3b, 0x60, 0x69, 0x5e, 0x5f, 0x64, 0x5d, 0x62, 0x63, 0x48, 0x71, 0x46, 0x47, 0x4c, 0x45, 0x4a, 0x4b, 0x70, 0x79, 0x6e,
                            0x6f, 0x74, 0x6d, 0x72, 0x73, 0x18, 0x41, 0x16, 0x17, 0x1c, 0x15, 0x1a, 0x1b, 0x40, 0x49, 0x3e, 0x3f, 0x44, 0x3d, 0x42, 0x43, 0x28, 0x51, 0x26, 0x27, 0x2c, 0x25, 0x2a, 0x2b,
                            0x50, 0x59, 0x4e, 0x4f, 0x54, 0x4d, 0x52, 0x53, 0x78, 0x21, 0x76, 0x77, 0x7c, 0x75, 0x7a, 0x7b, 0x20, 0x29, 0x1e, 0x1f, 0x24, 0x1d, 0x22, 0x23, 0x88, 0x31, 0x86, 0x87, 0x0c};
const byte lcdMode2[256] = {0x8a, 0x0f, 0x10, 0x35, 0x3e, 0x33, 0x34, 0x39, 0x32, 0x37, 0x38, 0x5d, 0x86, 0x5b, 0x5c, 0x61, 0x5a, 0x5f, 0x60, 0x85, 0x8e, 0x83, 0x84, 0x89, 0x82, 0x87,
                            0x88, 0x6d, 0x16, 0x6b, 0x6c, 0x71, 0x6a, 0x6f, 0x70, 0x15, 0x1e, 0x13, 0x14, 0x19, 0x12, 0x17, 0x18, 0x3d, 0x66, 0x3b, 0x3c, 0x41, 0x3a, 0x3f, 0x40, 0x65, 0x6e, 0x63, 0x64,
                            0x69, 0x62, 0x67, 0x68, 0x4d, 0x76, 0x4b, 0x4c, 0x51, 0x4a, 0x4f, 0x50, 0x75, 0x7e, 0x73, 0x74, 0x79, 0x72, 0x77, 0x78, 0x1d, 0x46, 0x1b, 0x1c, 0x21, 0x1a, 0x1f, 0x20, 0x45,
                            0x4e, 0x43, 0x44, 0x49, 0x42, 0x47, 0x48, 0x2d, 0x56, 0x2b, 0x2c, 0x31, 0x2a, 0x2f, 0x30, 0x55, 0x5e, 0x53, 0x54, 0x59, 0x52, 0x57, 0x58, 0x7d, 0x26, 0x7b, 0x7c, 0x81, 0x7a,
                            0x7f, 0x80, 0x25, 0x2e, 0x23, 0x24, 0x29, 0x22, 0x27, 0x28, 0x8d, 0x36, 0x8b, 0x8c, 0x11, 0x8a, 0x0f, 0x10, 0x35, 0x3e, 0x33, 0x34, 0x39, 0x32, 0x37, 0x38, 0x5d, 0x86, 0x5b,
                            0x5c, 0x61, 0x5a, 0x5f, 0x60, 0x85, 0x8e, 0x83, 0x84, 0x89, 0x82, 0x87, 0x88, 0x6d, 0x16, 0x6b, 0x6c, 0x71, 0x6a, 0x6f, 0x70, 0x15, 0x1e, 0x13, 0x14, 0x19, 0x12, 0x17, 0x18,
                            0x3d, 0x66, 0x3b, 0x3c, 0x41, 0x3a, 0x3f, 0x40, 0x65, 0x6e, 0x63, 0x64, 0x69, 0x62, 0x67, 0x68, 0x4d, 0x76, 0x4b, 0x4c, 0x51, 0x4a, 0x4f, 0x50, 0x75, 0x7e, 0x73, 0x74, 0x79,
                            0x72, 0x77, 0x78, 0x1d, 0x46, 0x1b, 0x1c, 0x21, 0x1a, 0x1f, 0x20, 0x45, 0x4e, 0x43, 0x44, 0x49, 0x42, 0x47, 0x48, 0x2d, 0x56, 0x2b, 0x2c, 0x31, 0x2a, 0x2f, 0x30, 0x55, 0x5e,
                            0x53, 0x54, 0x59, 0x52, 0x57, 0x58, 0x7d, 0x26, 0x7b, 0x7c, 0x81, 0x7a, 0x7f, 0x80, 0x25, 0x2e, 0x23, 0x24, 0x29, 0x22, 0x27, 0x28, 0x8d, 0x36, 0x8b, 0x8c, 0x11};

//////------------------------------------
//////------------------------------------
////// Setups

class BLEServerCallback : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("BLE connected");
    deviceConnected = true;

    if (blePicclyVisible)
    {
      bleLock = false;
      Serial.println(" ==> device connected ==> UNLOCK decision");
      Serial.println("-------------------------------------");
    }
    else
    {
      bleLock = false;
      Serial.println(" ==> device connected but PICLLY invisible ==> UNLOCK decision");
      Serial.println("-------------------------------------");
    }
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("BLE disonnected");
    deviceConnected = false;

    if (blePicclyVisible)
    {
      bleLock = false;

      Serial.println(" ==> device disconnected but PICLLY visible ==> UNLOCK decision");
      Serial.println("-------------------------------------");
    }
    else
    {
      bleLock = true;

      Serial.println(" ==> device disconnected but PICLLY invisible ==> LOCK decision");
      Serial.println("-------------------------------------");
    }
  }
};

class BLEAdvertisedDeviceCallback : public BLEAdvertisedDeviceCallbacks
{
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    //Serial.print("BLE Advertised Device found: ");
    //Serial.println(advertisedDevice.toString().c_str());
    /*
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(SERVICE_UUID)) {

      //
      Serial.print("Found our device!  address: ");
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;

    } // Found our server
    */
  } // onResult
};  // MyAdvertisedDeviceCallbacks

class BLECharacteristicCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    if (pCharacteristic->getUUID().toString() == MODE_CHARACTERISTIC_UUID)
    {
      std::string rxValue = pCharacteristic->getValue();
      modeOrder = rxValue[0];

      char print_buffer[500];
      sprintf(print_buffer, "%02x", modeOrder);
      Serial.print("Write mode : ");
      Serial.println(print_buffer);

      //      Serial.print("Notify mode : ");
      //      pCharacteristic->setValue((uint8_t *)&orderMode, 1);
      //      pCharacteristic->notify();
    }
  }

  void onRead(BLECharacteristic *pCharacteristic)
  {

    if (pCharacteristic->getUUID().toString() == MODE_CHARACTERISTIC_UUID)
    {
      pCharacteristicMode->setValue((uint8_t *)&modeOrder, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", modeOrder);
      Serial.print("Read mode : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == SPEED_CHARACTERISTIC_UUID)
    {
      pCharacteristicSpeed->setValue((uint8_t *)&speedCurrent, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", speedCurrent);
      Serial.print("Read speed : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == BRAKE_STATUS_CHARACTERISTIC_UUID)
    {
      pCharacteristicBrakeSentOrder->setValue((uint8_t *)&breakeSentOrder, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", breakeSentOrder);
      Serial.print("Read breakeSentOrder : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == VOLTAGE_STATUS_CHARACTERISTIC_UUID)
    {
      uint32_t voltage = voltageFilter.getMean();
      pCharacteristicVoltageStatus->setValue((uint8_t *)&voltage, 4);

      char print_buffer[500];
      sprintf(print_buffer, "%02f", voltage / 1000.0);
      Serial.print("Read voltage : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == CURRENT_STATUS_CHARACTERISTIC_UUID)
    {
      int32_t current = currentFilter.getMean();
      pCharacteristicCurrentStatus->setValue((uint8_t *)&current, 4);

      char print_buffer[500];
      sprintf(print_buffer, "%02d", current);
      Serial.print("Read current : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == POWER_STATUS_CHARACTERISTIC_UUID)
    {
      int32_t current = currentFilter.getMean();
      uint32_t voltage = voltageFilter.getMean();
      int32_t power = (current / 1000.0) * (voltage / 1000.0);
      pCharacteristicPowerStatus->setValue((uint8_t *)&power, 4);

      char print_buffer[500];
      sprintf(print_buffer, "%04d", power);
      Serial.print("Read power : ");
      Serial.println(print_buffer);
    }
    else if (pCharacteristic->getUUID().toString() == BTLOCK_STATUS_CHARACTERISTIC_UUID)
    {
      pCharacteristicBtlockStatus->setValue((uint8_t *)&bleLock, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", bleLock);
      Serial.print("Read bleLock : ");
      Serial.println(print_buffer);
    }
  }
};

void bleOnScanResults(BLEScanResults scanResults)
{
  Serial.print("BLE Scan Device found: ");
  Serial.println(scanResults.getCount());

  bool newBlePicclyVisible = false;

  for (int i = 0; i < scanResults.getCount(); i++)
  {
    String name = scanResults.getDevice(i).getName().c_str();
    int rssi = scanResults.getDevice(i).getRSSI();
    std::string address = scanResults.getDevice(i).getAddress().toString();
    String addressStr = address.c_str();

#if DEBUG_BLE_SCAN
    Serial.print("BLE device : ");
    Serial.print(name);
    Serial.print(" / adress : ");
    Serial.print(addressStr);
    Serial.print(" / name : ");
    Serial.print(name);
    Serial.print(" / rssi ");
    Serial.println(rssi);
#endif

    if (addressStr == "ac:23:3f:56:ec:6c")
    {
      if (rssi < -80)
      {
#if DEBUG_BLE_SCAN
        Serial.println(" ==> PICC-LY found ... but too far away ==> lock from scan");
        newBlePicclyVisible = false;
#endif
      }
      else
      {
#if DEBUG_BLE_SCAN
        Serial.println(" ==> PICC-LY found ==> unlock from scan");
#endif
        newBlePicclyVisible = true;
      }
    }
  }

  // store piclyy status
  blePicclyVisible = newBlePicclyVisible;

  // launch new scan
  pBLEScan->start(20, &bleOnScanResults, false);

  // set BT lock
  if ((!deviceConnected))
  {
    if (!blePicclyVisible)
    {
      bleLock = true;
      Serial.println(" ==> no device connected and PICC-LY no found ==> LOCK decision");
    }
    else
    {
      bleLock = false;
      Serial.println(" ==> no device connected and PICC-LY found ==> UNLOCK decision");
    }
  }

  Serial.println("-------------------------------------");
}

void setupPins()
{
  pinMode(PIN_IN_BREAK, INPUT);
  pinMode(PIN_IN_VOLTAGE, INPUT);
  pinMode(PIN_IN_CURRENT, INPUT);
}

void setupBLE()
{
  // Create the BLE Device
  BLEDevice::init("SmartLCD");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLEServerCallback());

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30);

  // Create a BLE Characteristic
  pCharacteristicSpeed = pService->createCharacteristic(
      SPEED_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicMode = pService->createCharacteristic(
      MODE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicBrakeSentOrder = pService->createCharacteristic(
      BRAKE_STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicVoltageStatus = pService->createCharacteristic(
      VOLTAGE_STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicCurrentStatus = pService->createCharacteristic(
      CURRENT_STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicPowerStatus = pService->createCharacteristic(
      POWER_STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicBtlockStatus = pService->createCharacteristic(
      BTLOCK_STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicSpeed->addDescriptor(new BLE2902());
  pCharacteristicMode->addDescriptor(new BLE2902());
  pCharacteristicBrakeSentOrder->addDescriptor(new BLE2902());
  pCharacteristicVoltageStatus->addDescriptor(new BLE2902());
  pCharacteristicCurrentStatus->addDescriptor(new BLE2902());
  pCharacteristicPowerStatus->addDescriptor(new BLE2902());
  pCharacteristicBtlockStatus->addDescriptor(new BLE2902());

  pCharacteristicSpeed->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicMode->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicBrakeSentOrder->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicVoltageStatus->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicCurrentStatus->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicPowerStatus->setCallbacks(new BLECharacteristicCallback());
  pCharacteristicBtlockStatus->setCallbacks(new BLECharacteristicCallback());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  // Start scanning
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new BLEAdvertisedDeviceCallback());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(20, &bleOnScanResults, false);
}

void setupSerial()
{

  //  swSerLcdToCntrl.begin(BAUD_RATE, SWSERIAL_8N1, PIN_SERIAL_LCD_TO_CNTRL_RX, SERIAL_LCD_TO_CNTRL_TXPIN, false, 256);
  hwSerLcdToCntrl.begin(BAUD_RATE, SERIAL_8N1, PIN_SERIAL_LCD_TO_CNTRL_RX, SERIAL_LCD_TO_CNTRL_TXPIN); // -> works

  //  swSerCntrlToLcd.begin(BAUD_RATE, SWSERIAL_8N1, SERIAL_CNTRL_TO_LCD_RXPIN, SERIAL_CNTRL_TO_LCD_TXPIN, false, 256);
  hwSerCntrlToLcd.begin(BAUD_RATE, SERIAL_8N1, SERIAL_CNTRL_TO_LCD_RXPIN, SERIAL_CNTRL_TO_LCD_TXPIN);
}

////// Setup
void setup()
{

  // Initialize the Serial (use only in setup codes)
  Serial.begin(115200);
  Serial.println(PSTR("\n\nsetup begin"));

  timeLastNotifyBle = millis();

  setupSerial();
  setupBLE();
  setupPins();

  // End off setup
  Serial.println("setup ok");
}

//////------------------------------------
//////------------------------------------
////// Various functions

uint8_t getCheckSum(char *string)
{
  byte rtn = 0;

  for (byte i = 0; i < 14; i++)
  {
    rtn ^= string[i];
  }

  return rtn;
}

void displayFrame(int mode, char data_buffer[], byte checksum)
{

  char print_buffer[500];

  // for excel
  sprintf(print_buffer, "(%d) %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x / %02x",
          mode,
          data_buffer[0],
          data_buffer[1],
          data_buffer[2],
          data_buffer[3],
          data_buffer[4],
          data_buffer[5],
          data_buffer[6],
          data_buffer[7],
          data_buffer[8],
          data_buffer[9],
          data_buffer[10],
          data_buffer[11],
          data_buffer[12],
          data_buffer[13],
          data_buffer[14],
          checksum);

  Serial.println(print_buffer);
}

void displaySpeed()
{

  Serial.print("speedCurrent : ");
  Serial.print(speedCurrent);
  Serial.println("");
}

void displayMode(char data_buffer[])
{

  uint32_t byteDiff = (data_buffer[5] - data_buffer[2]);
  uint8_t lcdMode = byteDiff & 0x03;
  uint8_t lcdMode2 = (byteDiff >> 2) & 0x1;
  uint8_t lcdMode3 = (byteDiff >> 3) & 0x1;
  uint8_t lcdMode4 = (byteDiff >> 4) & 0x1;
  uint8_t lcdMode5 = (byteDiff >> 5) & 0x1;
  uint8_t lcdMode6 = (byteDiff >> 6) & 0x1;
  uint8_t lcdMode7 = (byteDiff >> 7) & 0x1;

  char print_buffer[500];
  sprintf(print_buffer, "%02x %02x / %02x / %02x / %02x / %02x / %02x / %02x / %02x", data_buffer[2], data_buffer[5], lcdMode, lcdMode2, lcdMode3, lcdMode4, lcdMode5, lcdMode6, lcdMode7);

  Serial.print("LCD mode : ");
  Serial.print(print_buffer);
  Serial.println("");
}

uint8_t modifyModeOld(char var, char data_buffer[])
{
  uint32_t byteDiff = (var - data_buffer[2]);
  uint8_t lcdMode = byteDiff & 0x03;
  uint8_t lcdModeMask = byteDiff & 0xfc;
  uint8_t newLcdMode2 = modeOrder | lcdModeMask;
  uint32_t newLcdMode3 = (newLcdMode2 + data_buffer[2]) & 0xff;

  char print_buffer[500];
  /*
  sprintf(print_buffer, "%02x %02x / %s %02x / %s %02x / %s %02x / %s %02x  / %s %02x  / %s %02x ",
          data_buffer[2],
          var,
          "byteDiff",
          byteDiff,
          "lcd",
          lcdMode,
          "mask",
          lcdModeMask,
          "order",
          orderMode,
          "newLcdMode2",
          newLcdMode2,
          "newLcdMode3",
          newLcdMode3);
          */

  sprintf(print_buffer, "%s %02x / %s %02x",
          "lcd",
          lcdMode,
          "order",
          modeOrder);

  Serial.print("LCD mode : ");
  Serial.print(print_buffer);
  Serial.println("");

  return newLcdMode3;
}

uint8_t modifyMode(char var, char data_buffer[])
{
  uint8_t newLcdMode3;

  if (modeOrder == 0)
    newLcdMode3 = lcdMode0[(uint8_t)(data_buffer[2])];
  else if (modeOrder == 1)
    newLcdMode3 = lcdMode1[(uint8_t)(data_buffer[2])];
  else
    newLcdMode3 = lcdMode2[(uint8_t)(data_buffer[2])];

  return newLcdMode3;
}

uint8_t modifyPower(char var, char data_buffer[])
{
  uint8_t newPower;

  // lock escooter by reducing power to 5%
  if (bleLock == true)
    newPower = 10;
  else
    newPower = var;

  return newPower;
}

uint8_t modifyBrake(char var, char data_buffer[])
{
  /*
  uint8_t brakeStatusNew = !digitalRead(PIN_IN_BREAK);

  if ((brakeStatusNew == 1) && (brakeStatusOld == 0))
  {
    brakeStatus = brakeStatusNew;
    timeLastBrake = timeLoop;

    Serial.print("Brake pressed at : ");
    Serial.println(timeLastBrake);

    // notify bluetooth
    pCharacteristicBrakeSentOrder->setValue((uint8_t *)&breakeSentOrder, 1);
    pCharacteristicBrakeSentOrder->notify();
  }
  else if ((brakeStatusNew == 0) && (brakeStatusOld == 1))
  {
    brakeStatus = brakeStatusNew;

    // reset to min
    breakeSentOrder = breakeMin;

    Serial.print("Brake released at : ");
    Serial.println(timeLoop);

    // notify bluetooth
    pCharacteristicBrakeSentOrder->setValue((uint8_t *)&breakeSentOrder, 1);
    pCharacteristicBrakeSentOrder->notify();
  }

  if (brakeStatus == 1)
  {
    if (timeLoop - timeLastBrake > 1500)
    {
      breakeSentOrder = breakeMin + 3;
    }
    else if (timeLoop - timeLastBrake > 1000)
    {
      breakeSentOrder = breakeMin + 2;
    }
    else if (timeLoop - timeLastBrake > 500)
    {
      breakeSentOrder = breakeMin + 1;
    }
    else
    {
      breakeSentOrder = breakeMin;
    }

    // notify bluetooth
    pCharacteristicBrakeSentOrder->setValue((uint8_t *)&breakeSentOrder, 1);
    pCharacteristicBrakeSentOrder->notify();
  }
  else
  {
    breakeSentOrder = breakeMin;
  }
  brakeStatusOld = brakeStatusNew;

  char print_buffer[500];
  sprintf(print_buffer, "%s %02x %s %02x %s %02x %s %d %s %d",
          "Brake Status New : ",
          brakeStatusNew,
          " / breakeSentOrder  : ",
          breakeSentOrder,
          " / Current LCD brake  : ",
          var,
          " / timeLastBrake  : ",
          timeLastBrake,
          " / timeLoop  : ",
          timeLoop);

  Serial.println(print_buffer);
*/
  return breakeSentOrder;
}

uint8_t decodeOldSpeed()
{
  uint8_t high1 = (data_speed_buffer[2] - data_speed_buffer[0]) & 0xff;
  uint8_t offset_regul = (data_speed_buffer[1] - data_speed_buffer[0]) & 0xff;
  uint8_t high2 = (high1 - offset_regul) & 0xff;
  uint8_t low = (data_speed_buffer[3] - data_speed_buffer[0]);

  int speed = (((int)high2 * 256) + (low)) / 20.5;

  return speed;
}

uint8_t modifySpeedHigh(char var, char data_buffer[], int fakeSpeed)
{

  // modify speed
  if (speedOld > fakeSpeed - 1)
  {
    // modify speed
    return ((0x01 + data_buffer[3]) & 0xff);
  }
  else
  {
    return var;
  }
}

uint8_t modifySpeedLow(char var, char data_buffer[], int fakeSpeed)
{

  // modify speed
  if (speedOld > fakeSpeed - 1)
  {
    return ((0xf0 + data_buffer[3]) & 0xff);
  }
  else
  {
    return var;
  }
}

int readHardSerial(int i, HardwareSerial *ss, int mode, char data_buffer[])
{

  byte var;

  /*
    //Serial.print(i);
    //Serial.print(" / ");
  */

  if (ss->available() > 0)
  {

    var = ss->read();

    // LCD -> CNTRL
    if (mode == MODE_LCD_TO_CNTRL)
    {
      if ((var == 0xAA) && (begin_LcdToCntrl == 1))
      {
        begin_LcdToCntrl = 0;
        Serial.println(PSTR(" ===> detect begin AA"));
        i = 0;
      }
    }
    else
    // CNTRL -> LCD
    {
      if ((var == 0x36) && (begin_CntrlToLcd == 1))
      {
        begin_CntrlToLcd = 0;
        Serial.println(PSTR(" ===> detect begin 36"));
        i = 0;
      }
    }

    //---------------------
    // MODIFY LCD_TO_CNTRL
    if ((i == 5) && (mode == MODE_LCD_TO_CNTRL))
    {
      //var = modifyMode(var, data_buffer);
      var = modifyMode(var, data_buffer);
      isModified_LcdToCntrl = 1;
    }

    if ((i == 7) && (mode == MODE_LCD_TO_CNTRL))
    {
      var = modifyPower(var, data_buffer);
      isModified_LcdToCntrl = 1;
    }

    if ((i == 10) && (mode == MODE_LCD_TO_CNTRL))
    {
      var = modifyBrake(var, data_buffer);
      isModified_LcdToCntrl = 1;
    }

    //---------------------
    // MODIFY CNTRL_TO_LCD

    // modify speed
    if ((i == 7) && (mode == MODE_CNTRL_TO_LCD))
    {
      data_speed_buffer[0] = data_buffer[3];
      data_speed_buffer[1] = data_buffer[5];
      data_speed_buffer[2] = var;
      var = modifySpeedHigh(var, data_buffer, fakeSpeed);
      isModified_CntrlToLcd = 1;
    }
    if ((i == 8) && (mode == MODE_CNTRL_TO_LCD))
    {
      data_speed_buffer[3] = var;
      var = modifySpeedLow(var, data_buffer, fakeSpeed);
      isModified_CntrlToLcd = 1;

      speedOld = speedCurrent;
      speedCurrent = decodeOldSpeed();
    }

    // CHECKSUM
    if ((isModified_LcdToCntrl == 1) && (i == 14) && (mode == MODE_LCD_TO_CNTRL))
    {
      uint8_t oldChecksum = var;
      var = getCheckSum(data_buffer);

#if DEBUG_SERIAL_CHECKSUM_LCD_TO_CNTRL
      char print_buffer[500];
      sprintf(print_buffer, "%02x %02x",
              oldChecksum,
              var);

      Serial.print(" ===> modified checksum LCD_TO_CNTRL : ");
      Serial.println(print_buffer);
#endif

      isModified_LcdToCntrl = 0;
    }
    else if (((isModified_CntrlToLcd) == 1) && (i == 14) && (mode == MODE_CNTRL_TO_LCD))
    {

      uint8_t oldChecksum = var;
      var = getCheckSum(data_buffer);

#if DEBUG_SERIAL_CHECKSUM_CNTRL_TO_LCD
      char print_buffer[500];
      sprintf(print_buffer, "%02x %02x",
              oldChecksum,
              var);

      Serial.print(" ===> modified checksum CNTRL_TO_LCD : ");
      Serial.println(print_buffer);
#endif

      isModified_CntrlToLcd = 0;
    }

    data_buffer[i] = var;

    ss->write(var);

    // Serial.print(var < 0x10 ? PSTR(" 0") : PSTR(" "));
    // Serial.print(var, HEX);
    // Serial.print(" / ");
    // Serial.print(i);
    // Serial.println("");

    // display
    if (i == 14)
    {

      uint8_t checksum = getCheckSum(data_buffer);

      //Serial.println("");

      if (mode == MODE_CNTRL_TO_LCD)
      {
#if DEBUG_DISPLAY_FRAME_CNTRL_TO_LCD
        displayFrame(mode, data_buffer, checksum);
#endif
#if DEBUG_DISPLAY_SPEED
        displaySpeed();
#endif
      }
      else
      {
#if DEBUG_DISPLAY_FRAME_LCD_TO_CNTRL
        displayFrame(mode, data_buffer, checksum);
#endif
#if DEBUG_DISPLAY_MODE
        displayMode(data_buffer);
#endif
      }

      if (checksum != data_buffer[14])
      {
        Serial.println("====> CHECKSUM error ==> reset");

        if (mode == MODE_LCD_TO_CNTRL)
          begin_LcdToCntrl = 1;
        else
          begin_CntrlToLcd = 1;
      }

      i = 0;
    }
    else
    {
      i++;
    }
  }

  return i;
}

void processSerial()
{
  // read/write LCD -> CNTRL
  //  i_LcdToCntrl = readSoftSerial(i_LcdToCntrl, &swSerLcdToCntrl, MODE_LCD_TO_CNTRL, data_buffer_lcd);
  i_LcdToCntrl = readHardSerial(i_LcdToCntrl, &hwSerLcdToCntrl, MODE_LCD_TO_CNTRL, data_buffer_lcd);

  //i_CntrlToLcd = readSoftSerial(i_CntrlToLcd, &swSerCntrlToLcd, MODE_CNTRL_TO_LCD, data_buffer_cntrl);
  i_CntrlToLcd = readHardSerial(i_CntrlToLcd, &hwSerCntrlToLcd, MODE_CNTRL_TO_LCD, data_buffer_cntrl);
}

void processBLE()
{
  // notify changed value
  if (deviceConnected)
  {
    if (millis() > timeLastNotifyBle + 500)
    {

      pCharacteristicSpeed->setValue((uint8_t *)&speedCurrent, 1);
      pCharacteristicSpeed->notify();

#if DEBUG_BLE_NOTIFY
      Serial.print("Notify speed : ");
      Serial.println(speedCurrent);
#endif

      // notify bluetooth
      uint32_t voltage = voltageFilter.getMean();
      pCharacteristicVoltageStatus->setValue((uint8_t *)&voltage, 4);
      pCharacteristicVoltageStatus->notify();

#if DEBUG_BLE_NOTIFY
      Serial.print("Notify voltage : ");
      Serial.println(voltage);
#endif

      int32_t current = currentFilter.getMean();
      pCharacteristicCurrentStatus->setValue((uint8_t *)&current, 4);
      pCharacteristicCurrentStatus->notify();

#if DEBUG_BLE_NOTIFY
      Serial.print("Notify current : ");
      Serial.println(current);
#endif

      int32_t power = (current / 1000.0) * (voltage / 1000.0);
      if (power < 0)
        power = 0;
      pCharacteristicPowerStatus->setValue((uint8_t *)&power, 4);
      pCharacteristicPowerStatus->notify();

#if DEBUG_BLE_NOTIFY
      Serial.print("Notify power : ");
      Serial.println(power);
#endif

      pCharacteristicBtlockStatus->setValue((uint8_t *)&bleLock, 1);
      pCharacteristicBtlockStatus->notify();

#if DEBUG_BLE_NOTIFY
      Serial.print("Notify bleLock : ");
      Serial.println(bleLock);
#endif

      timeLastNotifyBle = millis();
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    //delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void processBrake()
{

  brakeStatus = digitalRead(PIN_IN_BREAK);

  /*Serial.print("Brake : ");
  Serial.println(brakeStatus);*/
}

void processVoltage()
{

  voltageStatus = analogRead(PIN_IN_VOLTAGE);

  voltageInMilliVolts = (voltageStatus * 1000) / ANALOG_TO_VOLTS;
  voltageFilter.in(voltageInMilliVolts);

  /*   Serial.print("Voltage read : ");
  Serial.print(voltageStatus);
  Serial.print(" / in volts : ");
  Serial.println(voltageInMilliVolts / 1000.0); */
}

void processCurrent()
{
  int curerntRead = analogRead(PIN_IN_CURRENT);
  int currentInMillamps = (curerntRead - ANALOG_TO_CURRENT_MOYENNE_ZERO) * 1000 / ANALOG_TO_CURRENT;
  currentFilter.in(currentInMillamps);

  /*
  Serial.print("Current read : ");
  Serial.print(curerntRead);
  Serial.print(" / in amperes : ");
  Serial.println(currentInMillamps / 1000.0);
  */
}
//////------------------------------------
//////------------------------------------
////// Main loop

void loop()
{

  processSerial();
  processBLE();

  if (i_loop % 100 == 1)
  {
    processVoltage();
    //processCurrent();
  }
  if (i_loop % 10 == 1)
  {
    processCurrent();
  }

  // Give a time for ESP
  delay(1);
  i_loop++;

  timeLoop = millis();
}

/////////// End