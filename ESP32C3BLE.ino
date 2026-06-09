

/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <BLEDevice.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <U8g2lib.h> //.42 inch OLED screen, assuming we're using the uhhhhhhhh esp32c3

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
// #define AIN1 25
// #define BIN1 27
// #define AIN2 26
// #define BIN2 14
// #define PWMA 19
// #define PWMB 18
// #define STBY 5  

#define ARDUINO_USB_CDC_ON_BOOT 1
#define ARDUINO_USB_MODE 1

#define AIN1 0
#define AIN2 1
#define PWMA 2
#define BIN1 3
#define BIN2 4
#define SDA_PIN 5
#define SCL_PIN 6
#define STBY 7 
#define PWMB 8
#define TRIG_PIN 9 // Trig pin connected to pin 0, PURPLE WIRE
#define ECHO_PIN 10 // Echo pin connected to pin 1, BLUE WIRE


#define bleServerName "CBLE"

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

static BLEUUID bmeServiceUUID("E329C6C0-927D-4CAA-8A26-F5D12C4FFB56");
static BLEUUID flexSensorUUID("63DD4869-49C5-4DB9-9734-3B215114E31F");

static boolean doConnect = false;
static boolean connected = false;
static BLEAddress *pServerAddress;
static BLERemoteCharacteristic* flexCharacteristic;

typedef struct {
    float volt_load;
    float theta;
} FloatPacket;


float flexVolt = 0;
float theta = 0;      
bool newVolt = false;

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// ---------------- TIMING ----------------
bool isMoving = false;
unsigned long moveStartTime = 0;
const unsigned long moveDuration = 3000; // 3 seconds

// Called when PSoC 6 sends a notification
static void flexNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                                uint8_t* pData, size_t length, bool isNotify) {
  if (length == sizeof(FloatPacket)) {
    FloatPacket packet;
    memcpy(&packet, pData, sizeof(FloatPacket));

    flexVolt = packet.volt_load;
    theta = packet.theta;
    newVolt = true;
  }
}

// Called when a BLE device is found during scan
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) {
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("Device found. Connecting!");
    }
  }
};

bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();

  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return false;
  }

  flexCharacteristic = pRemoteService->getCharacteristic(flexSensorUUID);
  if (flexCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic UUID");
    return false;
  }
  Serial.println(" - Found characteristic");

  flexCharacteristic->registerForNotify(flexNotifyCallback);
  Serial.println(" - Subscribed to notifications");
  u8g2.clearBuffer();
  u8g2.drawStr(0,10,"BLE ON!"); 
  u8g2.sendBuffer();

  delay(500); // <-- give PSoC time to receive and process the CCCD write

  return true;
}

int PWM_from_Volt(float volt) {
  //linear function for PWM vs V_load_resistor
  //no y int
  return volt;
}

int PWM_after_theta(int PWM, float theta) {
  const int m = 5;
  PWM = PWM - (m * theta);
  if (PWM <= 0) {
    return 0;
  }
  else return PWM;
}

const float distanceThreshold = 30.48;

float getDistance() {
  // 2 us pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send trigger signal
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure time for echo to return
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert time to distance 
  float distance = duration * 0.034 / 2; // this is in CM
  return distance;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting ESP32 BLE Central...");
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,10,"BLE OFF!"); 
  u8g2.sendBuffer();  

  BLEDevice::init("");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}


void loop()
{
  if (doConnect) {
    connectToServer(*pServerAddress);
    doConnect = false;
    connected = true;
  }

  if (newVolt) {
    newVolt = false;
    // Serial.print("Received: ");
    // Serial.println(flexVolt);
    // Serial.println(theta);
    int max_pwm_val = PWM_from_Volt(flexVolt);
    u8g2.clearBuffer();

    char buffer[10];
    itoa(max_pwm_val, buffer, 10);

    int pwm_l = max_pwm_val;
    int pwm_r = max_pwm_val;

    if (theta < 0) {
      pwm_l = PWM_after_theta(max_pwm_val , -theta);
    }
    else {
      pwm_r = PWM_after_theta(max_pwm_val , theta);
    }
    Serial.print("PWM L AND RIGHT:");
    Serial.println(pwm_l);
    Serial.println(pwm_r);
    motor1.drive(pwm_l);
    motor2.drive(pwm_r);
    u8g2.drawStr(0,10, "BLE ON! ");
    u8g2.drawStr(0,40, "Throttle: ");
    u8g2.drawStr(50,40, buffer);
    u8g2.sendBuffer();  

  }
    float distance = getDistance();
    flexCharacteristic->writeValue((uint16_t)distance, true);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(10);
}

