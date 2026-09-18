#include <Arduino.h>
#include <SPI.h>

#include <RF24.h>
#include <LoRa.h>

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/*
   ============================================================
                     MINEGUARD ROVER
   ============================================================

   ESP32 DevKit V1

   COMMUNICATION
   ------------------------------------------------------------
   nRF24L01 = remote-control receiver
   LoRa     = long-range telemetry

   SENSORS
   ------------------------------------------------------------
   MQ-4      = methane response
   MQ-7      = CO response
   BME280    = temperature / humidity / pressure
   MPU6050   = acceleration / gyroscope
   VL53L0X   = ToF distance
   HC-SR04   = ultrasonic distance
   OLED      = local display

   MOTOR
   ------------------------------------------------------------
   L298N
   Left motor  = OUT1 / OUT2
   Right motor = OUT3 / OUT4

   ============================================================
*/


// ============================================================
// nRF24L01
// ============================================================

#define NRF_CE   16
#define NRF_CSN  17

RF24 radio(NRF_CE, NRF_CSN);

const byte RADIO_ADDRESS[6] = "ROVER";


// ============================================================
// LoRa
// ============================================================

#define LORA_SS    32
#define LORA_RST   2
#define LORA_DIO0  33

// CHANGE THIS to the actual frequency of your LoRa module.
// 433E6 for a 433-MHz module.
// 868E6 for an 868-MHz module.
#define LORA_FREQUENCY 433E6


// ============================================================
// GAS SENSORS
// ============================================================

#define MQ4_PIN 36
#define MQ7_PIN 39


// ============================================================
// I2C
// ============================================================

#define I2C_SDA 21
#define I2C_SCL 22


// ============================================================
// HC-SR04
// ============================================================

#define HCSR04_TRIG 18
#define HCSR04_ECHO 19


// ============================================================
// L298N
// ============================================================

#define MOTOR_LEFT_IN1   25
#define MOTOR_LEFT_IN2   26

#define MOTOR_RIGHT_IN1  27
#define MOTOR_RIGHT_IN2  14


// ============================================================
// ALERTS
// ============================================================

#define BUZZER_PIN 4

/*
   GPIO1 and GPIO3 are UART0 pins on classic ESP32.
   They are therefore NOT used for Serial in this program.
*/
#define GREEN_LED 1
#define RED_LED   3


// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// ============================================================
// SENSOR OBJECTS
// ============================================================

Adafruit_BME280 bme;

Adafruit_MPU6050 mpu;

Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();


// ============================================================
// CONTROL PACKET
// ============================================================

struct ControlPacket {

  int16_t x;
  int16_t y;

  uint8_t up;
  uint8_t down;
  uint8_t left;
  uint8_t right;
  uint8_t mode;
  uint8_t joystickButton;

  uint16_t sequence;
};

ControlPacket command;


// ============================================================
// FAILSAFE
// ============================================================

unsigned long lastRadioPacket = 0;

const unsigned long RADIO_TIMEOUT = 500;


// ============================================================
// SENSOR VALUES
// ============================================================

float temperature = 0;
float humidity = 0;
float pressure = 0;

float accelerationX = 0;
float accelerationY = 0;
float accelerationZ = 0;

uint16_t tofDistance = 8190;

float ultrasonicDistance = -1;

int mq4Raw = 0;
int mq7Raw = 0;


// ============================================================
// MOTOR FUNCTIONS
// ============================================================

void stopMotors()
{
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);

  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}


void leftForward()
{
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}


void leftBackward()
{
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
}


void rightForward()
{
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
}


void rightBackward()
{
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
}


void moveForward()
{
  leftForward();
  rightForward();
}


void moveBackward()
{
  leftBackward();
  rightBackward();
}


void turnLeft()
{
  leftBackward();
  rightForward();
}


void turnRight()
{
  leftForward();
  rightBackward();
}


// ============================================================
// APPLY REMOTE COMMAND
// ============================================================

void processRemoteCommand()
{
  /*
     Priority:

     STOP / joystick neutral
     explicit buttons
     joystick
  */

  if (command.joystickButton) {
    stopMotors();
    return;
  }

  // Explicit directional buttons
  if (command.up) {
    moveForward();
    return;
  }

  if (command.down) {
    moveBackward();
    return;
  }

  if (command.left) {
    turnLeft();
    return;
  }

  if (command.right) {
    turnRight();
    return;
  }

  // Joystick control
  const int DEADZONE = 350;

  int x = command.x;
  int y = command.y;

  bool xActive = abs(x) > DEADZONE;
  bool yActive = abs(y) > DEADZONE;

  if (!xActive && !yActive) {
    stopMotors();
    return;
  }

  if (y < -DEADZONE) {
    moveForward();
  }
  else if (y > DEADZONE) {
    moveBackward();
  }
  else if (x < -DEADZONE) {
    turnLeft();
  }
  else if (x > DEADZONE) {
    turnRight();
  }
  else {
    stopMotors();
  }
}


// ============================================================
// READ nRF24
// ============================================================

void receiveRemote()
{
  if (radio.available()) {

    while (radio.available()) {
      radio.read(&command, sizeof(command));
    }

    lastRadioPacket = millis();

    processRemoteCommand();
  }

  // HARD FAILSAFE
  if (millis() - lastRadioPacket > RADIO_TIMEOUT) {
    stopMotors();
  }
}


// ============================================================
// HC-SR04
// ============================================================

float readUltrasonic()
{
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(HCSR04_TRIG, LOW);

  unsigned long duration =
      pulseIn(HCSR04_ECHO, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343f / 2.0f;
}


// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
  mq4Raw = analogRead(MQ4_PIN);

  mq7Raw = analogRead(MQ7_PIN);


  temperature = bme.readTemperature();

  humidity = bme.readHumidity();

  pressure = bme.readPressure() / 100.0f;


  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(
    &accel,
    &gyro,
    &temp
  );

  accelerationX = accel.acceleration.x;
  accelerationY = accel.acceleration.y;
  accelerationZ = accel.acceleration.z;


  VL53L0X_RangingMeasurementData_t measurement;

  vl53.rangingTest(
    &measurement,
    false
  );

  if (measurement.RangeStatus != 4) {
    tofDistance = measurement.RangeMilliMeter;
  }
  else {
    tofDistance = 8190;
  }


  ultrasonicDistance =
      readUltrasonic();
}


// ============================================================
// OLED
// ============================================================

void updateDisplay()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);

  display.print("MINEGUARD ROVER");

  display.setCursor(0, 12);

  display.print("T:");
  display.print(temperature, 1);

  display.print("C H:");
  display.print(humidity, 0);

  display.print("%");


  display.setCursor(0, 24);

  display.print("MQ4:");
  display.print(mq4Raw);

  display.print(" MQ7:");
  display.print(mq7Raw);


  display.setCursor(0, 36);

  display.print("TOF:");

  if (tofDistance < 8190) {
    display.print(tofDistance);
    display.print("mm");
  }
  else {
    display.print("---");
  }


  display.setCursor(0, 48);

  display.print("US:");

  if (ultrasonicDistance >= 0) {
    display.print(ultrasonicDistance, 0);
    display.print("cm");
  }
  else {
    display.print("---");
  }


  display.display();
}


// ============================================================
// SEND TELEMETRY THROUGH LORA
// ============================================================

void sendTelemetry()
{
  /*
     This is deliberately a simple text telemetry packet.
     The remote-control link remains nRF24.
  */

  LoRa.beginPacket();

  LoRa.print("T=");
  LoRa.print(temperature, 1);

  LoRa.print(",H=");
  LoRa.print(humidity, 1);

  LoRa.print(",P=");
  LoRa.print(pressure, 1);

  LoRa.print(",MQ4=");
  LoRa.print(mq4Raw);

  LoRa.print(",MQ7=");
  LoRa.print(mq7Raw);

  LoRa.print(",TOF=");
  LoRa.print(tofDistance);

  LoRa.print(",US=");
  LoRa.print(ultrasonicDistance, 1);

  LoRa.print(",AX=");
  LoRa.print(accelerationX, 2);

  LoRa.print(",AY=");
  LoRa.print(accelerationY, 2);

  LoRa.print(",AZ=");
  LoRa.print(accelerationZ, 2);

  LoRa.endPacket();
}


// ============================================================
// ALERT SYSTEM
// ============================================================

void updateAlerts()
{
  /*
     These are demonstration thresholds only.
     Do NOT interpret MQ sensor raw ADC values as calibrated
     methane/CO concentrations.
  */

  bool danger = false;

  /*
     Conservative prototype trigger:
     unusually high raw sensor response.
     Calibrate these experimentally before using them.
  */

  if (mq4Raw > 3000) {
    danger = true;
  }

  if (mq7Raw > 3000) {
    danger = true;
  }


  if (danger) {

    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);

    digitalWrite(BUZZER_PIN, HIGH);

  }
  else {

    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);

    digitalWrite(BUZZER_PIN, LOW);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  /*
     IMPORTANT:
     No Serial.begin() is used because GPIO1/GPIO3 are being
     used for LEDs in this pin allocation.
  */


  // ---------------- MOTORS ----------------

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);

  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  stopMotors();


  // ---------------- ALERTS ----------------

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);


  // ---------------- GAS ----------------

  analogReadResolution(12);

  pinMode(MQ4_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);


  // ---------------- HC-SR04 ----------------

  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);

  digitalWrite(HCSR04_TRIG, LOW);


  // ---------------- I2C ----------------

  Wire.begin(I2C_SDA, I2C_SCL);


  // ---------------- BME280 ----------------

  bool bmeOK = bme.begin(0x76);

  if (!bmeOK) {
    bmeOK = bme.begin(0x77);
  }


  // ---------------- MPU6050 ----------------

  mpu.begin(0x68);


  // ---------------- VL53L0X ----------------

  vl53.begin();


  // ---------------- OLED ----------------

  display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDRESS
  );

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);

  display.setCursor(0, 0);

  display.println("MINEGUARD");

  display.println("Starting...");

  display.display();


  // ---------------- SPI ----------------

  SPI.begin(
    5,    // SCK
    13,   // MISO
    23,   // MOSI
    -1
  );


  // ========================================================
  // nRF24 INITIALIZATION
  // ========================================================

  if (!radio.begin()) {

    display.clearDisplay();

    display.setCursor(0, 0);

    display.println("nRF24 ERROR");

    display.display();

    while (true) {
      stopMotors();
      delay(1000);
    }
  }


  radio.setAutoAck(true);

  radio.setRetries(3, 5);

  radio.setChannel(108);

  radio.setDataRate(RF24_250KBPS);

  radio.setPALevel(RF24_PA_LOW);

  radio.setPayloadSize(sizeof(ControlPacket));

  radio.openReadingPipe(
    1,
    RADIO_ADDRESS
  );

  radio.startListening();


  // ========================================================
  // LoRa INITIALIZATION
  // ========================================================

  LoRa.setPins(
    LORA_SS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(LORA_FREQUENCY)) {

    display.clearDisplay();

    display.setCursor(0, 0);

    display.println("LoRa ERROR");

    display.display();

    /*
       nRF remote control can still operate.
       Therefore don't permanently stop the rover here.
    */

  }


  lastRadioPacket = millis();

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println("MINEGUARD READY");

  display.display();

  delay(1000);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  static unsigned long lastSensorRead = 0;
  static unsigned long lastDisplay = 0;
  static unsigned long lastTelemetry = 0;


  // --------------------------------------------------------
  // REMOTE CONTROL
  // --------------------------------------------------------

  receiveRemote();


  // --------------------------------------------------------
  // SENSOR SAMPLING
  // --------------------------------------------------------

  if (millis() - lastSensorRead >= 250) {

    lastSensorRead = millis();

    readSensors();

    updateAlerts();
  }


  // --------------------------------------------------------
  // OLED
  // --------------------------------------------------------

  if (millis() - lastDisplay >= 500) {

    lastDisplay = millis();

    updateDisplay();
  }


  // --------------------------------------------------------
  // LORA TELEMETRY
  // --------------------------------------------------------

  if (millis() - lastTelemetry >= 1000) {

    lastTelemetry = millis();

    sendTelemetry();
  }
}