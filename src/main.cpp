//////------------------------------------
////// Inludes

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//////------------------------------------
////// Defines

#define SERIAL_LCD_TO_CNTRL_RXPIN 25
#define SERIAL_LCD_TO_CNTRL_TXPIN 26

#define SERIAL_CNTRL_TO_LCD_RXPIN 34
#define SERIAL_CNTRL_TO_LCD_TXPIN 13

#define PIN_READ_FLAG 33
#define PIN_WRITE_FLAG 32
#define PIN_IN_BREAK 12

#define MODE_LCD_TO_CNTRL 0
#define MODE_CNTRL_TO_LCD 1

#define DATA_BUFFER_SIZE 30
#define BAUD_RATE 1200

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SPEED_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define MODE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

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

HardwareSerial hwSerCntrlToLcd(1);
HardwareSerial hwSerLcdToCntrl(2);

int i_loop = 0;

int i_LcdToCntrl = 0;
int i_CntrlToLcd = 0;
int begin_LcdToCntrl = 1;
int begin_CntrlToLcd = 1;

int isModified_LcdToCntrl = 0;

uint8_t currentSpeed = 0;
uint8_t orderMode = 1;

uint8_t brakeStatus = 0;
uint8_t brakeStatusOld = 0;
uint8_t breakeMin = 2;
uint8_t breakeMax = 4;
uint8_t breakeOrder = breakeMin;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicSpeed = NULL;
BLECharacteristic *pCharacteristicMode = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEScan *pBLEScan;

const byte lcdMode0[256] = {0x80, 0x5, 0x6, 0x2b, 0x34, 0x29, 0x2a, 0x2f, 0x28, 0x2d, 0x2e, 0x53, 0x7c, 0x51, 0x52, 0x57, 0x50, 0x55, 0x56, 0x7b, 0x84, 0x79, 0x7a, 0x7f, 0x78,
                            0x7d, 0x7e, 0x63, 0x0c, 0x61, 0x62, 0x67, 0x60, 0x65, 0x66, 0x0b, 0x14, 0x9, 0x0a, 0x0f, 0x8, 0x0d, 0x0e, 0x33, 0x5c, 0x31, 0x32, 0x37, 0x30, 0x35, 0x36, 0x5b, 0x64, 0x59, 0x5a,
                            0x5f, 0x58, 0x5d, 0x5e, 0x43, 0x6c, 0x41, 0x42, 0x47, 0x40, 0x45, 0x46, 0x6b, 0x74, 0x69, 0x6a, 0x6f, 0x68, 0x6d, 0x6e, 0x13, 0x3c, 0x11, 0x12, 0x17, 0x10, 0x15, 0x16, 0x3b,
                            0x44, 0x39, 0x3a, 0x3f, 0x38, 0x3d, 0x3e, 0x23, 0x4c, 0x21, 0x22, 0x27, 0x20, 0x25, 0x26, 0x4b, 0x54, 0x49, 0x4a, 0x4f, 0x48, 0x4d, 0x4e, 0x73, 0x1c, 0x71, 0x72, 0x77, 0x70,
                            0x75, 0x76, 0x1b, 0x24, 0x19, 0x1a, 0x1f, 0x18, 0x1d, 0x1e, 0x83, 0x2c, 0x81, 0x82, 0x7, 0x80, 0x5, 0x6, 0x2b, 0x34, 0x29, 0x2a, 0x2f, 0x28, 0x2d, 0x2e, 0x53, 0x7c, 0x51, 0x52,
                            0x57, 0x50, 0x55, 0x56, 0x7b, 0x84, 0x79, 0x7a, 0x7f, 0x78, 0x7d, 0x7e, 0x63, 0x0c, 0x61, 0x62, 0x67, 0x60, 0x65, 0x66, 0x0b, 0x14, 0x9, 0x0a, 0x0f, 0x8, 0x0d, 0x0e, 0x33, 0x5c,
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
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("BLE disonnected");
    deviceConnected = false;
  }
};

class BLEAdvertisedDeviceCallback : public BLEAdvertisedDeviceCallbacks
{
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // Serial.print("BLE Advertised Device found: ");
    // Serial.println(advertisedDevice.toString().c_str());
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
      orderMode = rxValue[0];

      char print_buffer[500];
      sprintf(print_buffer, "%02x", orderMode);
      Serial.print("Write mode : ");
      Serial.println(print_buffer);
    }
  }

  void onRead(BLECharacteristic *pCharacteristic)
  {
    if (pCharacteristic->getUUID().toString() == MODE_CHARACTERISTIC_UUID)
    {
      pCharacteristic->setValue((uint8_t *)&orderMode, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", orderMode);
      Serial.print("Read mode : ");
      Serial.println(print_buffer);
    }

    if (pCharacteristic->getUUID().toString() == SPEED_CHARACTERISTIC_UUID)
    {
      pCharacteristic->setValue((uint8_t *)&currentSpeed, 1);

      char print_buffer[500];
      sprintf(print_buffer, "%02x", currentSpeed);
      Serial.print("Read speed : ");
      Serial.println(print_buffer);
    }
  }
};

void bleOnScanResults(BLEScanResults scanResults)
{
  // Serial.print("BLE Scan Device found: ");
  // Serial.println(scanResults.getCount());
  // for (int i = 0; i < scanResults.getCount(); i++) {
  //   String name = scanResults.getDevice(i).getName().c_str();
  //   int rssi = scanResults.getDevice(i).getRSSI();
  //   Serial.print("BLE device : ");
  //   Serial.print(name);
  //   Serial.print(" / ");
  //   Serial.println(rssi);
  // }
  pBLEScan->start(20, &bleOnScanResults, false);
}

void setupPins()
{
  pinMode(PIN_IN_BREAK, INPUT);
}

void setupBLE()
{
  // Create the BLE Device
  BLEDevice::init("SmartLCD");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLEServerCallback());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristicSpeed = pService->createCharacteristic(
      SPEED_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_READ);

  pCharacteristicMode = pService->createCharacteristic(
      MODE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_READ);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristicSpeed->addDescriptor(new BLE2902());
  pCharacteristicMode->addDescriptor(new BLE2902());
  pCharacteristicMode->setCallbacks(new BLECharacteristicCallback());

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

  //  swSerLcdToCntrl.begin(BAUD_RATE, SWSERIAL_8N1, SERIAL_LCD_TO_CNTRL_RXPIN, SERIAL_LCD_TO_CNTRL_TXPIN, false, 256);
  hwSerLcdToCntrl.begin(BAUD_RATE, SERIAL_8N1, SERIAL_LCD_TO_CNTRL_RXPIN, SERIAL_LCD_TO_CNTRL_TXPIN); // -> works

  //  swSerCntrlToLcd.begin(BAUD_RATE, SWSERIAL_8N1, SERIAL_CNTRL_TO_LCD_RXPIN, SERIAL_CNTRL_TO_LCD_TXPIN, false, 256);
  hwSerCntrlToLcd.begin(BAUD_RATE, SERIAL_8N1, SERIAL_CNTRL_TO_LCD_RXPIN, SERIAL_CNTRL_TO_LCD_TXPIN);

  pinMode(PIN_READ_FLAG, OUTPUT);
  pinMode(PIN_WRITE_FLAG, OUTPUT);
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
  // char print_buffer[20];

  byte rtn = 0;

  for (byte i = 0; i < 14; i++)
  {

    // Serial.print("i.:");
    // Serial.print(i);
    // Serial.print(" - ");

    rtn ^= string[i];

    // sprintf(print_buffer, "%02x", rtn);
    // Serial.print(print_buffer);
    // Serial.print(" / ");
  }

  // Serial.println("");
  return rtn;
}

void displayFrame(int mode, char data_buffer[], byte checksum)
{

  char print_buffer[500];

  // pour excel
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
  /* 
              sprintf(print_buffer, "%d - %02x %02x %02x %02x %02x %02x %02x %02x %02x /// %02x",
                      mode,
                      (data_buffer[4] - data_buffer[3])  & 0xff,
                      (data_buffer[5] - data_buffer[3])  & 0xff,
                      (data_buffer[7] - data_buffer[3])  & 0xff,
                      (data_buffer[8] - data_buffer[3])  & 0xff,
                      (data_buffer[9] - data_buffer[3])  & 0xff,
                      (data_buffer[10] - data_buffer[3])  & 0xff,
                      (data_buffer[11] - data_buffer[3])  & 0xff,
                      (data_buffer[12] - data_buffer[3])  & 0xff,
                      (data_buffer[13] - data_buffer[3])  & 0xff,
                      (data_buffer[7] - data_buffer[5])  & 0xff
                     );
*/
  //debugE("%s", print_buffer);
  Serial.println(print_buffer);
}

void displaySpeed(char data_buffer[])
{

  uint8_t high1 = (data_buffer[7] - data_buffer[3]) & 0xff;
  uint8_t offset_regul = (data_buffer[5] - data_buffer[3]) & 0xff;
  uint8_t high2 = (high1 - offset_regul) & 0xff;
  uint8_t low = (data_buffer[8] - data_buffer[3]);

  int speed = (((int)high2 * 256) + (low)) / 20.5;

  currentSpeed = speed;

  /*
  char print_buffer[500];
    sprintf(print_buffer, "%02x %02x %02x / %4d %4d %4d / %4d",
            (data_buffer[5] - data_buffer[3]) & 0xff,
            (data_buffer[7] - data_buffer[3]) & 0xff,
            (data_buffer[8] - data_buffer[3]) & 0xff,
            high1,
            high2,
            low,
            speed);
    //debugE("%s", print_buffer);
    Serial.println(print_buffer);
            */

  Serial.print("speed : ");
  Serial.print(speed);
  Serial.print(" / regulator : ");
  Serial.print(offset_regul != 0);
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

uint8_t modifyMode(char var, char data_buffer[])
{
  uint32_t byteDiff = (var - data_buffer[2]);
  uint8_t lcdMode = byteDiff & 0x03;
  uint8_t lcdModeMask = byteDiff & 0xfc;
  uint8_t newLcdMode2 = orderMode | lcdModeMask;
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
          orderMode);

  Serial.print("LCD mode : ");
  Serial.print(print_buffer);
  Serial.println("");

  return newLcdMode3;
}

uint8_t modifyMode2(char var, char data_buffer[])
{
  uint8_t newLcdMode3;

  if (orderMode == 0)
    newLcdMode3 = lcdMode0[(uint8_t)(data_buffer[2])];
  else if (orderMode == 1)
    newLcdMode3 = lcdMode1[(uint8_t)(data_buffer[2])];
  else
    newLcdMode3 = lcdMode2[(uint8_t)(data_buffer[2])];

  return newLcdMode3;
}

uint8_t modifyBrake(char var, char data_buffer[])
{

  uint8_t brakeStatusNew = !digitalRead(PIN_IN_BREAK);

  if ((brakeStatusNew == 1) && (brakeStatusOld == 0))
  {
    brakeStatus = brakeStatusNew;
    timeLastBrake = timeLoop;

    Serial.print("Brake pressed at : ");
    Serial.println(timeLastBrake);
  }
  else if ((brakeStatusNew == 0) && (brakeStatusOld == 1))
  {
    brakeStatus = brakeStatusNew;

    Serial.print("Brake released at : ");
    Serial.println(timeLoop);
  }

  if (brakeStatus == 1)
  {
    if (timeLoop - timeLastBrake > 1500)
    {
      breakeOrder = breakeMin + 3;
    }
    else if (timeLoop - timeLastBrake > 1000)
    {
      breakeOrder = breakeMin + 2;
    }
    else if (timeLoop - timeLastBrake > 500)
    {
      breakeOrder = breakeMin + 1;
    }
    else
    {
      breakeOrder = breakeMin;
    }
  }
  else
  {
    breakeOrder = breakeMin;
  }

  brakeStatusOld = brakeStatusNew;

  char print_buffer[500];
  sprintf(print_buffer, "%s %02x %s %02x %s %02x %s %d %s %d",
          "Brake Status New : ",
          brakeStatusNew,
          " / Order LCD brake  : ",
          breakeOrder,
          " / Current LCD brake  : ",
          var,
          " / timeLastBrake  : ",
          timeLastBrake,
          " / timeLoop  : ",
          timeLoop);

  Serial.println(print_buffer);

  return breakeOrder;
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

    digitalWrite(PIN_READ_FLAG, HIGH);
    var = ss->read();
    digitalWrite(PIN_READ_FLAG, LOW);

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

    if ((i == 5) && (mode == MODE_LCD_TO_CNTRL))
    {
      //var = modifyMode(var, data_buffer);
      var = modifyMode2(var, data_buffer);
      isModified_LcdToCntrl = 1;
    }

    if ((i == 10) && (mode == MODE_LCD_TO_CNTRL))
    {
      var = modifyBrake(var, data_buffer);
      isModified_LcdToCntrl = 1;
    }

    if ((isModified_LcdToCntrl == 1) && (i == 14) && (mode == MODE_LCD_TO_CNTRL))
    {
      uint8_t oldChecksum = var;
      var = getCheckSum(data_buffer);

      char print_buffer[500];
      sprintf(print_buffer, "%02x %02x",
              oldChecksum,
              var);

      //Serial.print(" ===> modified checksum : ");
      //Serial.println(print_buffer);

      isModified_LcdToCntrl = 0;
    }

    data_buffer[i] = var;

    digitalWrite(PIN_WRITE_FLAG, HIGH);
    ss->write(var);
    digitalWrite(PIN_WRITE_FLAG, LOW);

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
        //displayFrame(mode, data_buffer, checksum);
        //displaySpeed(data_buffer);
      }
      else
      {
        //displayFrame(mode, data_buffer, checksum);
        //displayMode(data_buffer);
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
    if (millis() > timeLastNotifyBle + 100)
    {
      pCharacteristicSpeed->setValue((uint8_t *)&currentSpeed, 1);
      pCharacteristicSpeed->notify();
      Serial.println("notify");

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
  Serial.print("Brake : ");
  Serial.print(brakeStatus);
}
//////------------------------------------
//////------------------------------------
////// Main loop

void loop()
{

  if (i_loop % 1000 == 1)
  {
    //    Serial.println("alive");
  }

  processSerial();
  processBLE();

  // Give a time for ESP
  delay(1);
  i_loop++;

  timeLoop = millis();
}

/////////// End