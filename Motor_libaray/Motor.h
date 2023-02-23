#ifndef MOTOR_HPP_
#define MOTOR_HPP_

class Motor
{
public:
  Motor(int motor_number, int frequency);
  ~Motor();
  void run(int);
  void set_speed(int);
};

#endif