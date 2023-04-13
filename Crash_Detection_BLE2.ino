#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
//#define USE_SPI       // Uncomment this to use SPI

#define SERVICE_UUID        "c4f8792f-a07b-433b-a48a-187136d4e8b5"
#define CHARACTERISTIC_UUID "dff10e08-2c91-40a1-8a75-22a1cd583c09"

#define SERIAL_PORT Serial

//#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

BLECharacteristic *pCharacteristic = NULL;
BLEServer *pServer = NULL;
BLEService *pService = NULL;

int fsr1AnalogPin = 36; // FSR1 is connected to GPIO 36 (ADC1_CH0)
int fsr2AnalogPin = 34; // FSR2 is connected to GPIO 34 (ADC1_CH6)
int fsr3AnalogPin = 39; // FSR3 is connected to GPIO 39 (ADC1_CH3)

int fsr1Reading;      // the analog reading from the FSR resistor divider
int fsr2Reading;
int fsr3Reading;
bool deviceConnected = false;
uint8_t crash = 0;
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
// TODO May need to modify firmware to notice changes to deviceConnected
// and restart advertising when any such change happens
void setup()
{
  SERIAL_PORT.begin(115200);
  
  BLEDevice::init("Helmet");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY                                         
                                       );

  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setValue(&crash,1);
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
  pAdvertising->start(); // maybe pServer->startAdvertising(); 

  pinMode(A0, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A6, OUTPUT);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(1000);
    }
    else
    {
      initialized = true;
    }
  }
}

void loop()
{
  fsr1Reading = analogRead(fsr1AnalogPin);
  Serial.print("FSR 1 Analog reading = ");
  Serial.println(fsr1Reading);
  if (fsr1Reading >= 300){ 
    Serial.println("CRASH DETECTED!");
    ESP32BLE_notify ();  
  }
  fsr2Reading = analogRead(fsr2AnalogPin);
  Serial.print("FSR 2 Analog reading = ");
  Serial.println(fsr2Reading);
  if (fsr2Reading >= 300){
    Serial.println("CRASH DETECTED!");
    ESP32BLE_notify ();
  }
 fsr3Reading = analogRead(fsr3AnalogPin);
  Serial.print("FSR 3 Analog reading = ");
  Serial.println(fsr3Reading);
  if (fsr3Reading >= 300){
    Serial.println("CRASH DETECTED!");
    ESP32BLE_notify ();
  }
  delay(10);  
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(1000);
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(1000);
  }
  delay(200);
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
  if (abs(sensor->gyrY())>10)
  { 
    SERIAL_PORT.print("CRASH DETECTED!");
    SERIAL_PORT.println();
    ESP32BLE_notify ();
  }
}



void ESP32BLE_notify ()
{
  if (crash == 0) {    
  crash = 7;
  SERIAL_PORT.println("Attempting BLE notification.");
  SERIAL_PORT.println("setValue");
  // NOTE: second arg in next line is not value but size in bytes
  pCharacteristic->setValue(&crash,1); // TODO: allow client to acknowledge (and reset)
  SERIAL_PORT.println("notify");  
  pCharacteristic->notify();
  SERIAL_PORT.println("start");  
  
  delay (200);
  crash = 0;
  }
}
