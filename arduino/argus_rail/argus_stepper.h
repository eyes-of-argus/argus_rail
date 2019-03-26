#ifndef ARGUS_STEPPER_H
#define ARGUS_STEPPER_H

#define INWARD  HIGH
#define OUTWARD LOW

enum Moves {
  FROM3TO2,
  FROM3TO1,
  FROM2TO3,
  FROM2TO1,
  FROM1TO3,
  FROM1TO2
};

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
    digitalWrite(_enable_pin, HIGH);
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
  void setStepCount(int full_move_steps)
  {
    _full_move = full_move_steps;
    _half_move = (_full_move / 2);
  }
  void setRequestedMove(Moves requested_move)
  {
    _requested_move = requested_move;
  }
  void requestMove(uint8_t requested_position)
  {
    uint8_t current_position = this->getPosition();
    if(requested_position != current_position)
    {
      if(requested_position == 3)
      {
        if(current_position == 2) this->setRequestedMove(FROM2TO3);
        else if(current_position == 1) this->setRequestedMove(FROM1TO3);
      }
      else if(requested_position == 2)
      {
        if(current_position == 3) this->setRequestedMove(FROM3TO2);
        else if(current_position == 1) this->setRequestedMove(FROM1TO2);
      }
      else if(requested_position == 1)
      {
        if(current_position == 3) this->setRequestedMove(FROM3TO1);
        else if(current_position == 2) this->setRequestedMove(FROM2TO1);
      }
      this->moveToPos();
    }
  }
private:
  uint8_t _pulse_pin, _direction_pin, _enable_pin;
  bool _enabled = LOW, _direction = LOW;
  int _pulse_length = 100;
  int _full_move = 30550, _half_move = (_full_move / 2);
  uint8_t _current_position = 3;
  Moves _requested_move;

  void move(int steps, bool direction, int speed)
  {
    this->setDirection(direction);
    this->setSpeed(speed);

    while(steps > 0)
    {
      digitalWrite(_pulse_pin, HIGH);
      delayMicroseconds(_pulse_length);
      digitalWrite(_pulse_pin, LOW);
      delayMicroseconds(_pulse_length);
      steps--;
    }
  }
  void moveToPos()
  {
    switch(_requested_move)
    {
      case FROM3TO2:
        this->move(this->_half_move, INWARD, this->_pulse_length);
        this->setPosition(2);
        break;
      case FROM3TO1:
        this->move(this->_full_move, INWARD, this->_pulse_length);
        this->setPosition(1);
        break;
      case FROM2TO3:
        this->move(this->_half_move, OUTWARD, this->_pulse_length);
        this->setPosition(3);
        break;
      case FROM2TO1:
        this->move(this->_half_move, INWARD, this->_pulse_length);
        this->setPosition(1);
        break;
      case FROM1TO3:
        this->move(this->_full_move, OUTWARD, this->_pulse_length);
        this->setPosition(3);
        break;
      case FROM1TO2:
        this->move(this->_half_move, OUTWARD, this->_pulse_length);
        this->setPosition(2);
        break;
    }
  }
};

#endif