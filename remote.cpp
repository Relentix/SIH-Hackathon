#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

/*
   ============================================================
   MINEGUARD REMOTE CONTROLLER
   ESP32 DevKit V1 + nRF24L01 + Joystick + Buttons

   nRF24L01:
     CE   -> GPIO5
     CSN  -> GPIO17
     SCK  -> GPIO18
     MOSI -> GPIO23
     MISO -> GPIO19
     VCC  -> 3.3V ONLY
     GND  -> GND

   Joystick:
     VRx -> GPIO34
     VRy -> GPIO35
     SW  -> GPIO32

   Buttons:
     UP     -> GPIO33
     DOWN   -> GPIO25
     LEFT   -> GPIO26
     RIGHT  -> GPIO27
     MODE   -> GPIO14
   ============================================================
*/

// ---------------- NRF24 ----------------
#define NRF_CE   5
#define NRF_CSN  17

RF24 radio(NRF_CE, NRF_CSN);

// Same address must be used by the rover receiver.
const byte RADIO_ADDRESS[6] = "ROVER";

// ---------------- JOYSTICK ----------------
#define JOY_X       34
#define JOY_Y       35
#define JOY_SW      32

// ---------------- BUTTONS ----------------
#define BTN_UP      33
#define BTN_DOWN    25
#define BTN_LEFT    26
#define BTN_RIGHT   27
#define BTN_MODE    14

// ---------------- JOYSTICK SETTINGS ----------------
const int ADC_MIN = 0;
const int ADC_MAX = 4095;

const int JOYSTICK_CENTER = 2048;
const int JOYSTICK_DEADZONE = 300;

// ---------------- PACKET ----------------
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

ControlPacket packet;

uint16_t sequenceNumber = 0;

unsigned long lastSendTime = 0;

const unsigned long SEND_INTERVAL = 25; // 40 packets/sec


// ============================================================
// READ JOYSTICK
// ============================================================

int16_t readJoystick(int pin)
{
  int raw = analogRead(pin);

  int value = raw - JOYSTICK_CENTER;

  if (abs(value) < JOYSTICK_DEADZONE) {
    value = 0;
  }

  value = constrain(value, -2048, 2047);

  return value;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  // Joystick
  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_SW, INPUT_PULLUP);

  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);

  // ADC configuration
  analogReadResolution(12);

  // SPI
  SPI.begin(18, 19, 23, NRF_CSN);

  // nRF24
  if (!radio.begin()) {
    // No serial dependency.
    while (true) {
      delay(1000);
    }
  }

  radio.setAutoAck(true);
  radio.setRetries(3, 5);

  radio.setChannel(108);

  radio.setDataRate(RF24_250KBPS);

  radio.setPALevel(RF24_PA_LOW);

  radio.setPayloadSize(sizeof(ControlPacket));

  radio.openWritingPipe(RADIO_ADDRESS);

  radio.stopListening();

  delay(100);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  unsigned long now = millis();

  if (now - lastSendTime < SEND_INTERVAL) {
    return;
  }

  lastSendTime = now;

  // Read joystick
  packet.x = readJoystick(JOY_X);
  packet.y = readJoystick(JOY_Y);

  // Read buttons
  packet.up   = !digitalRead(BTN_UP);
  packet.down = !digitalRead(BTN_DOWN);

  packet.left  = !digitalRead(BTN_LEFT);
  packet.right = !digitalRead(BTN_RIGHT);

  packet.mode = !digitalRead(BTN_MODE);

  packet.joystickButton = !digitalRead(JOY_SW);

  packet.sequence = sequenceNumber++;

  // Transmit
  radio.write(&packet, sizeof(packet));
}