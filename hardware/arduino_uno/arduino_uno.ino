#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// GPS TX connects to Arduino D4 (RX=4, TX=3)
SoftwareSerial gpsSerial(4, 3);
TinyGPSPlus gps;

// L298N pins (Remove ENA and ENB jumpers on L298N module)
const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 6;
const int IN3 = 10;
const int IN4 = 11;

// Safe PWM limits with strong turning boost so neither motor stalls on turns
const int PWM_MIN = 130;
const int PWM_DEFAULT = 175;
const int PWM_MAX = 215;
const int PWM_TURN_BOOST = 35; // Extra torque during Left/Right tank turns
const int PWM_KICK = 230;      // 160ms kickstart pulse on any direction change

int motorSpeed = PWM_DEFAULT;
char currentCmd = 'S';

unsigned long lastCommandTime = 0;
unsigned long lastGpsOutput = 0;
unsigned long kickEndTime = 0;
bool inKickstart = false;

void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  inKickstart = false;
}

void setMotorPins(int a, int b, int c, int d, int pwmA, int pwmB)
{
  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
  digitalWrite(IN3, c);
  digitalWrite(IN4, d);

  analogWrite(ENA, pwmA);
  analogWrite(ENB, pwmB);
}

void applyMovement(char cmd, int basePwm)
{
  int turnPwm = min(basePwm + PWM_TURN_BOOST, PWM_MAX);

  switch (cmd)
  {
  case 'F':
    setMotorPins(HIGH, LOW, HIGH, LOW, basePwm, basePwm);
    break;
  case 'B':
    setMotorPins(LOW, HIGH, LOW, HIGH, basePwm, basePwm);
    break;
  case 'L':
    setMotorPins(LOW, HIGH, HIGH, LOW, turnPwm, turnPwm);
    break;
  case 'R':
    setMotorPins(HIGH, LOW, LOW, HIGH, turnPwm, turnPwm);
    break;
  default:
    stopMotors();
    break;
  }
}

void handleMotorCommand(char newCmd)
{
  lastCommandTime = millis();

  if (newCmd == 'S')
  {
    currentCmd = 'S';
    stopMotors();
    return;
  }

  // Trigger a 160ms high-torque kickstart whenever changing direction (e.g. S->R or F->R)
  if (newCmd != currentCmd)
  {
    inKickstart = true;
    kickEndTime = millis() + 160;
    currentCmd = newCmd;
    applyMovement(currentCmd, PWM_KICK);
  }
  else
  {
    int activePwm = inKickstart ? PWM_KICK : motorSpeed;
    applyMovement(currentCmd, activePwm);
  }
}

void drainGpsSerial()
{
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }
}

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  stopMotors();

  // Wait 1.5s for ESP32 boot noise to settle while still reading GPS
  unsigned long bootStart = millis();
  while (millis() - bootStart < 1500)
  {
    drainGpsSerial();
    while (Serial.available())
    {
      Serial.read();
    }
  }

  // Brief boot self-test: 150ms Forward, 150ms Reverse so you see both directions work
  digitalWrite(LED_BUILTIN, HIGH);
  setMotorPins(HIGH, LOW, HIGH, LOW, PWM_DEFAULT, PWM_DEFAULT);
  delay(150);
  setMotorPins(LOW, HIGH, LOW, HIGH, PWM_DEFAULT, PWM_DEFAULT);
  delay(150);
  stopMotors();
  digitalWrite(LED_BUILTIN, LOW);

  lastCommandTime = millis();
}

void loop()
{
  // 1. Continuously drain GPS buffer so NMEA sentences never overflow or fail checksum
  drainGpsSerial();

  // 2. Read movement commands from ESP32
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S')
    {
      digitalWrite(LED_BUILTIN, (c != 'S') ? HIGH : LOW);
      handleMotorCommand(c);
    }
    else if (c >= '1' && c <= '9')
    {
      motorSpeed = map(c - '0', 1, 9, PWM_MIN, PWM_MAX);
      if (currentCmd != 'S' && !inKickstart)
      {
        applyMovement(currentCmd, motorSpeed);
      }
    }
    drainGpsSerial();
  }

  unsigned long now = millis();

  // 3. End kickstart pulse and settle to normal speed
  if (inKickstart && (long)(now - kickEndTime) >= 0)
  {
    inKickstart = false;
    if (currentCmd != 'S')
    {
      applyMovement(currentCmd, motorSpeed);
    }
  }

  // 4. Safety watchdog: stop motors if no command arrives for 1500 ms
  if (currentCmd != 'S' && (now - lastCommandTime > 1500))
  {
    currentCmd = 'S';
    digitalWrite(LED_BUILTIN, LOW);
    stopMotors();
  }

  // 5. Send GPS status to ESP32 once per second
  if (now - lastGpsOutput >= 1000)
  {
    lastGpsOutput = now;

    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (now > 6000 && gps.charsProcessed() < 10)
    {
      Serial.println("GPS,NO_DATA,CHECK_WIRING");
    }
    else if (gps.location.isValid() && gps.location.age() < 15000)
    {
      Serial.print("GPS,");
      Serial.print(gps.location.lat(), 6);
      Serial.print(",");
      Serial.print(gps.location.lng(), 6);
      Serial.print(",");
      Serial.print(sats);
      Serial.print(",");
      if (gps.hdop.isValid())
      {
        Serial.println(gps.hdop.hdop(), 1);
      }
      else
      {
        Serial.println("-1");
      }
    }
    else
    {
      Serial.print("GPS,NO_FIX,SAT=");
      Serial.println(sats);
    }
    drainGpsSerial();
  }
}
