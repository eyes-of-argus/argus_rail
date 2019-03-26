// Listens for number of steps to move via serial com port

#include "/home/argus/argus_ws/src/argus_rail/arduino/argus_rail/argus_stepper.h"

uint8_t requested_position = 3;

ArgusStepper Right(10, 9, 8);

void setup() {
  Serial.begin(115200);
  
  Right.begin();
  Right.setSpeed(25);
  Right.setDirection(INWARD);
  Right.setPosition(3);

  Serial.print("Initialized... Beginning position assume: ");
  Serial.println(Right.getPosition());
}

void loop() {  
  // Handle Serial
  if (Serial.available())
  {
    requested_position = Serial.parseInt();

    // returns 0 if error or no int found
    if(requested_position == 0)
    {
      requested_position = Right.getPosition();
      Serial.println("ERROR: Could not parse int");
    }
    else
    {
      Serial.print("Current: "); Serial.print(Right.getPosition());
      Serial.print("\tRequested: "); Serial.println(requested_position);
    }

    // Clear serial input buffer
    Serial.end();
    Serial.begin(115200);
  }

  // Handle requested move
  Right.requestMove(requested_position);
}
