// M. Kaan Tasbas | mktasbas@gmail.com
// March 2019

/*
 * Arduino Nano sketch for controling the camera rail on the Argus robot
 * 
 * Connections:
 * 
 * Camera    | Left | Right
 * pulse     | D06  | D10  // Active on rising edge, min width 2.5us
 * direction | D05  | D09  // HIGH: inward, LOW: outward
 * opto      | VCC  | VCC  // optocoupler power (+5V);
 * enable    | D04  | D08  // HIGH: enabled, LOW: disabled
 * 
 * Note: 5us delay req after changing direction or enable
 */

#define INWARD  HIGH
#define OUTWARD LOW

 class ArgusStepper {
  public:
    ArgusStepper(const uint8_t pulse_pin, const uint8_t direction_pin, const uint8_t enable_pin)
    {
      _pulse_pin = pulse_pin;
      _direction_pin = direction_pin;
      _enable_pin = enable_pin;
    }
    void begin(void)
    {
      pinMode(_pulse_pin, OUTPUT);
      pinMode(_direction_pin, OUTPUT);
      pinMode(_enable_pin, OUTPUT);
      digitalWrite(_enable_pin, LOW);
    }
    void setDirection(bool direction)
    {
      _direction = direction;
      digitalWrite(_direction_pin, _direction);
      delayMicroseconds(5);
    }
    void reverseDirection(void)
    {
      // true/high: inwards, false/low: outward
      _direction = !_direction;
      digitalWrite(_direction_pin, _direction);
      delayMicroseconds(5);
    }
    void setSpeed(const int pulse_length)
    {
      // higher value is slower, unit in us
      _pulse_length = pulse_length;
    }
    bool sendPulse(void)
    {
      // sends one full pulse with selected delay
      // blocking function, will take _pulse_delay*2 to return
      if(!_enabled) return false;
      digitalWrite(_pulse_pin, HIGH);
      delayMicroseconds(_pulse_length);
      digitalWrite(_pulse_pin, LOW);
      delayMicroseconds(_pulse_length);
      return true;
    }
    void enable(void)
    {
      _enabled = HIGH;
      digitalWrite(_enable_pin, _enabled);
      delayMicroseconds(5);
    }
    void disable(void)
    {
      _enabled = LOW;
      digitalWrite(_enable_pin, _enabled);
      delayMicroseconds(5);
    }
    void setPosition(uint8_t position)
    {
      _current_position = position;
    }
    uint8_t getPosition(void)
    {
      return _current_position;
    }
  private:
    uint8_t _pulse_pin, _direction_pin, _enable_pin;
    bool _enabled = LOW, _direction = LOW;
    int _pulse_length = 100;
    uint8_t _current_position = 3;
 };

ArgusStepper Left(6, 5, 4);

void setup() {
  Left.begin();
  Serial.begin(9600);
}

void loop() {
  Left.setSpeed(50);
  Left.setDirection(INWARD);
  Left.enable();
  
  // move from baseline 3 to 1
  for(int i = 0; i < 5000; ++i)
  {
    Left.sendPulse();
    Serial.println(i);
  }
  
  while(1);
}
