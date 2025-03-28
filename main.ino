#define FRONT_TRIG 5
#define FRONT_ECHO 4
#define LEFT_TRIG 14
#define LEFT_ECHO 13
#define RIGHT_TRIG 27
#define RIGHT_ECHO 26

#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

void setup() {
  Serial.begin(115200);

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  long frontDist = getDistance(FRONT_TRIG, FRONT_ECHO);
  long leftDist = getDistance(LEFT_TRIG, LEFT_ECHO);
  long rightDist = getDistance(RIGHT_TRIG, RIGHT_ECHO);

  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print("cm, Left: ");
  Serial.print(leftDist);
  Serial.print("cm, Right: ");
  Serial.println(rightDist);

  if (frontDist > 20) {
    moveForward();
  } else {
    stopRobot();
    delay(500);
    if (leftDist > rightDist) {
      turnLeft();
      delay(500);
    } else {
      turnRight();
      delay(500);
    }
  }
}
