// Motor A pins
int IN1 = 5;
int IN2 = 15;
int PWM_A = 2;

// Motor B pins
int IN3 = 18;
int IN4 = 19;
int PWM_B = 4;

// Base speed
int speed = 60;

// Turning factor (how much slower inner wheel is)
float turnFactor = 0.5;

void setup() {

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_A, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_B, OUTPUT);

}

// Forward
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

// Backward
void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

// Turn left (left wheel slower)
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(PWM_A, speed * turnFactor);
  analogWrite(PWM_B, speed);
}

// Turn right (right wheel slower)
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed * turnFactor);
}

void loop() {
  delay(5000)
  // forward
  speed = 90;
  forward();
  delay(3000);

  // turn left
  speed = 60;
  turnLeft();
  delay(2000);

  // forward faster
  speed = 220;
  forward();
  delay(3000);

}