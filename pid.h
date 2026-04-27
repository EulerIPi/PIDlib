#ifndef PID_H
#define PID_H

#include <float.h>
#define BUFFER_SIZE 500
class PID{
  private:
    uint16_t count = 0, fullBuffer = 0;
    float buffer[BUFFER_SIZE];
    float prop, integ = 0, der;
    float kp, ki, kd;
    float MAX[3];
    unsigned long timeAnt = 0, time;
    bool start = true;
    float errorAnt = 0, error, deltaT = 0;
    float auxInteg;
    float out;
    float setAnt = 0.0f, constReset = 9999.0;
    typedef enum{
      P,
      I,
      D,
      ALL
    }PID_TYPE;
    float PID::limitter(float val, float lim)
  public:
    PID(float kp, float ki, float kd, float constReset = 5000.0f, float MAXall = 9999.0f, float MAXp = 9999.0f, float MAXi = 9999.0f, float MAXd = 9999.0f);
    float get(float set, float ret);
};
#endif
