// Listens for number of steps to move via serial com port

#define PULSE_PIN   10
#define DIR_PIN     9
#define ENABLE_PIN  8

int speed_delay = 15;
long step_count = 0;
bool dir = HIGH;    // inward
bool enable = HIGH;  // disabled

void setup() {
  Serial.begin(115200);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, enable);
  delayMicroseconds(10);
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(10);
}

void loop() {
  // Read step count from Serial
  if (Serial.available())
  {
    step_count = Serial.parseInt();
    
    // check for negative (dir change)
    if(step_count != 0)
    {
      // negative value
      if(step_count < 0)
      {
        // reverse direction
        dir = LOW;
        step_count = abs(step_count);
      }
      else
      {
        dir = HIGH;
      }
      
      digitalWrite(DIR_PIN, dir);
      delayMicroseconds(10);

      Serial.print("Dir: "); Serial.print(dir);
      Serial.print("\tStep: "); Serial.println(step_count);
    }
    else
    {
      Serial.println("Error: Couldn't parse int");
    }

    // Clear serial input buffer
    Serial.end();
    Serial.begin(115200);
  }

  while(step_count > 0)
  {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(speed_delay);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(speed_delay);
    step_count--;
  }
}
