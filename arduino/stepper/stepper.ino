
const int step_pin = 6; //pwm
const int direction_pin = 3;
const int enable_pin = 5;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);

}

void loop() {
  static uint8_t enable = LOW;  // high: move, low: stop
  static int speed_delay = 50;  // higher: slower, in microseconds
  static uint8_t dir = LOW;     // high: cameras left, low: cameras right
  static long step_count = 0;    // track number of steps

  // Read enable setting from Serial
  if (Serial.available())
  {
    if (Serial.parseInt() == 1) enable = HIGH;
    else enable = LOW;

    // Print information to console
    Serial.print("Enable: ");
    Serial.println(enable);
    Serial.print("Step Count: ");
    Serial.println(step_count);

    // Clear serial input buffer
    Serial.end();
    Serial.begin(9600);

    // Change in direction so reset step counter
    step_count = 0;
  }

  // Set enable and direction pins
  digitalWrite(enable_pin, enable);
  delayMicroseconds(10);

  digitalWrite(direction_pin, dir);
  delayMicroseconds(10);

  if(enable)
  {
    // Send pulse
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(speed_delay);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(speed_delay - 20);
    step_count++;
  }
}
