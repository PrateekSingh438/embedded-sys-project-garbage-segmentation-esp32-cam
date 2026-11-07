#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// GPS TX connects to Arduino D4
SoftwareSerial gpsSerial(4, 3);
TinyGPSPlus gps;

// ESP32 UART uses Arduino hardware Serial (D0/D1)
const int ENA = 5;
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 6;
const int IN3 = 10;
const int IN4 = 11;

int motorSpeed = 150;
char command = 'S';

unsigned long lastCommandTime = 0;
unsigned long lastGpsOutput = 0;

void stopMotors()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotors(int a, int b, int c, int d)
{
  digitalWrite(IN1, a);
  digitalWrite(IN2, b);
  digitalWrite(IN3, c);
  digitalWrite(IN4, d);

  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

// If a direction is reversed on your robot, swap the two
// wires of that motor on the L298N (or swap its HIGH/LOW here).
void moveRobot(char cmd)
{
  switch (cmd)
  {
  case 'F':
    setMotors(HIGH, LOW, HIGH, LOW);
    break;

  case 'B':
    setMotors(LOW, HIGH, LOW, HIGH);
    break;

  case 'L':
    setMotors(LOW, HIGH, HIGH, LOW);
    break;

  case 'R':
    setMotors(HIGH, LOW, LOW, HIGH);
    break;

  default:
    stopMotors();
    break;
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

  stopMotors();
  lastCommandTime = millis();
}

void loop()
{
  // Read GPS data
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }

  // Receive movement commands from ESP32
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == 'F' || c == 'B' || c == 'L' ||
        c == 'R' || c == 'S')
    {
      command = c;
      lastCommandTime = millis();
      moveRobot(command);
    }
    else if (c >= '0' && c <= '9')
    {
      motorSpeed = map(c - '0', 0, 9, 0, 255);
      if (command != 'S')
      {
        moveRobot(command); // apply new speed immediately
      }
    }
  }

  // Stop if commands stop arriving
  if (millis() - lastCommandTime > 1000)
  {
    command = 'S';
    stopMotors();
  }

  // Send GPS status to ESP32 once per second
  if (millis() - lastGpsOutput >= 1000)
  {
    lastGpsOutput = millis();

    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      // Nothing is arriving from the GPS module at all
      Serial.println("GPS,NO_DATA,CHECK_WIRING");
    }
    else if (gps.location.isValid() && gps.location.age() < 3000)
    {
      Serial.print("GPS,");
      Serial.print(gps.location.lat(), 6);
      Serial.print(",");
      Serial.print(gps.location.lng(), 6);
      Serial.print(",");
      Serial.println(sats);
    }
    else
    {
      Serial.print("GPS,NO_FIX,SAT=");
      Serial.println(sats);
    }
  }
}