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

bool dir = HIGH;    // inward
bool enable = HIGH;  // enabled

enum Moves {
  STAY     = 0,
  FROM3TO2 = 1,
  FROM3TO1 = 2,
  FROM2TO3 = -1,
  FROM2TO1 = 1,
  FROM1TO3 = -2,
  FROM1TO2 = -1
};

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

  static Moves requested_move;
  
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
  if(requested_position != current_position)
  {
    if(requested_position == 3)
    {
      if(current_position == 2) requested_move = FROM2TO3;
      else if(current_position == 1) requested_move = FROM1TO3;
    }
    else if(requested_position == 2)
    {
      if(current_position == 3) requested_move = FROM3TO2;
      else if(current_position == 1) requested_move = FROM1TO2;
    }
    else if(requested_position == 1)
    {
      if(current_position == 3) requested_move = FROM3TO1;
      else if(current_position == 2) requested_move = FROM2TO1;
    }
    moveToPos(requested_move);
  }
}

void move(int steps, bool direction, int speed)
{
  // set direction
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
}

void moveToPos(Moves requested_move)
{
  switch(requested_move)
  {
    case FROM3TO2:
      move(HALF_MOVE, INWARD, STEP_SPEED);
      current_position = 2;
      break;
    case FROM3TO1:
      move(FULL_MOVE, INWARD, STEP_SPEED);
      current_position = 1;
      break;
    case FROM2TO3:
      move(HALF_MOVE, OUTWARD, STEP_SPEED);
      current_position = 3;
      break;
    case FROM2TO1:
      move(HALF_MOVE, INWARD, STEP_SPEED);
      current_position = 1;
      break;
    case FROM1TO3:
      move(FULL_MOVE, OUTWARD, STEP_SPEED);
      current_position = 3;
      break;
    case FROM1TO2:
      move(HALF_MOVE, OUTWARD, STEP_SPEED);
      current_position = 2;
      break;
  }
}
