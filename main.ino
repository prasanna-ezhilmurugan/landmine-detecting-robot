#include <TinyGPSPlus.h>

// Motor Control Pins
#define IN1 25 // Left Motors Forward
#define IN2 26 // Left Motors Reverse
#define IN3 27 // Right Motors Forward
#define IN4 14 // Right Motors Reverse
#define ENA 13 // PWM for Left Motors
#define ENB 12 // PWM for Right Motors

// Ultrasonic Sensor Pins
#define TRIG_FRONT 5
#define ECHO_FRONT 18
#define TRIG_LEFT 4
#define ECHO_LEFT 19
#define TRIG_RIGHT 2
#define ECHO_RIGHT 21

// GPS Module Pins
#define GPS_RX 16 // Connect to TX of GPS module
#define GPS_TX 17 // Connect to RX of GPS module

// Speed Control (0 to 255)
int motorSpeed = 255; // Normal speed
int turnSpeed = 120;  // Slower for turning

TinyGPSPlus gps;
HardwareSerial myGPS(1); // Use UART1 for ESP32

void setup() {
  Serial.begin(115200); // Debugging
  myGPS.begin(9600, SERIAL_8N1, GPS_RX,
              GPS_TX); // GPS baud rate, 8 data bits, no parity, 1 stop bit

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  stopMotors();
}

void loop() {
  int frontDist = getDistance(TRIG_FRONT, ECHO_FRONT);
  int leftDist = getDistance(TRIG_LEFT, ECHO_LEFT);
  int rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" cm, Left: ");
  Serial.print(leftDist);
  Serial.print(" cm, Right: ");
  Serial.println(rightDist);

  updateGPS();

  if (frontDist > 20) {
    forward();
  } else {
    stopMotors();
    delay(500);

    if (leftDist > rightDist) {
      turnLeft();
    } else {
      turnRight();
    }
  }
  delay(500);
}

// Get distance from ultrasonic sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(15); // Increased pulse duration
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0)
    return 400; // If no signal, assume max distance

  int distance = duration * 0.034 / 2;
  return distance;
}

// Move Forward
void forward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn Left
void turnLeft() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn Right
void turnRight() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Stop Motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// GPS Update Function
void updateGPS() {
  while (myGPS.available()) {
    gps.encode(myGPS.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("GPS Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.printl n(gps.location.lng(), 6);
  }
}
