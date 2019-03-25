#ifndef ARGUS_STEPPER_H
#define ARGUS_STEPPER_H

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
    void sendPulse(void)
    {
      // sends one full pulse with selected delay
      // blocking function, will take _pulse_delay*2 to return
      digitalWrite(_pulse_pin, HIGH);
      delayMicroseconds(_pulse_length);
      digitalWrite(_pulse_pin, LOW);
      delayMicroseconds(_pulse_length);
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
    bool isEnabled(void)
    {
      return _enabled;
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

#endif