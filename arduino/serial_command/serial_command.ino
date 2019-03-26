// Listens for number of steps to move via serial com port

#define PULSE_PIN   10
#define DIR_PIN     9
#define ENABLE_PIN  8

#define HALF_MOVE   15275
#define FULL_MOVE   30550

#define INWARD      HIGH
#define OUTWARD     LOW

#define STEP_SPEED  25

uint8_t current_position = 3;
uint8_t requested_position = current_position;

int speed_delay = 25;
long step_count = 15275;
bool dir = HIGH;    // inward
bool enable = HIGH;  // enabled


void setup() {
  Serial.begin(115200);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, enable);
  delayMicroseconds(10);
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(10);

  Serial.print("Initialized... Beginning position assume: ");
  Serial.println(current_position);
}

void loop() {
  
  // Handle Serial
  if (Serial.available())
  {
    requested_position = Serial.parseInt();

    // returns 0 if error or no int found
    if(requested_position == 0)
    {
      requested_position = current_position;
      Serial.println("ERROR: Could not parse int");
    }
    else
    {
      Serial.print("Current: "); Serial.print(current_position);
      Serial.print("\tRequested: "); Serial.println(requested_position);
    }

    // Clear serial input buffer
    Serial.end();
    Serial.begin(115200);
  }

  // Handle requested move
  if(requested_position == 3)
  {
    if(current_position == 3)
    {
      // do nothing
    }
    else if(current_position == 2)
    {
      // 2 TO 3
      move(HALF_MOVE, OUTWARD, STEP_SPEED);
      current_position = 3;
    }
    else if(current_position == 1)
    {
      // 1 TO 3
      move(FULL_MOVE, OUTWARD, STEP_SPEED);
      current_position = 3;
    }
  }
  else if(requested_position == 2)
  {
    if(current_position == 2)
    {
      // do nothing
    }
    else if(current_position == 3)
    {
      // 3 TO 2
      move(HALF_MOVE, INWARD, STEP_SPEED);
      current_position = 2;
    }
    else if(current_position == 1)
    {
      // 1 TO 2
      move(HALF_MOVE, OUTWARD, STEP_SPEED);
      current_position = 2;
    }
  }
  else if(requested_position == 1)
  {
    if(current_position == 1)
    {
      // do nothing
    }
    else if(current_position == 3)
    {
      // 3 TO 1
      move(FULL_MOVE, INWARD, STEP_SPEED);
      current_position = 1;
    }
    else if(current_position == 2)
    {
      // 2 TO 1
      move(HALF_MOVE, INWARD, STEP_SPEED);
      current_position = 1;
    }
  }

}

void move(int steps, bool direction, int speed)
{
  // enable stepper and set direction
//  digitalWrite(ENABLE_PIN, HIGH);
//  delayMicroseconds(10);
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(10);

  // move number of steps
  while(steps > 0)
  {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(speed);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(speed);
    steps--;
  }

  // Disable stepper
//  digitalWrite(ENABLE_PIN, LOW);
//  delayMicroseconds(10);
}
