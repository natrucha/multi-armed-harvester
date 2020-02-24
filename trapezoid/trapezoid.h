#include <Arduino.h>

// see algorithm in https://sam.ensam.eu/bitstream/handle/10985/11974/LSIS_CEP_2017_BEAREE.pdf?sequence=1&isAllowed=y

class trapezoid {
  private:
    float q0;    // initial location
    float qe;    // end location (position set point)
    float v0;    // adapted initial conditions for accel limited profile
    // kinematic bounds
    float v_max, a_max, d_max;
    // total distance needed to travel
    float Dq;
    // min stopping distance
    float Dq_stop;
    // min distance moved with nonzero stopping velocity
    float Dq_v0;
    // min distance for which max velocity is reached
    float Dq_v_max;
    // Total trap time
    float Tt;

    // calculate the sign of v0 for the trapezoidal profile function
    byte checkSign(float value);
    // calculate the trapezoidal profile times and reached values
    void trapProfile();
    // calculate the triangular profile times and reached values
    void triangProfile();

  public:
    // reached values of velocity and acceleration
    float vr, ar, dr;
    // duration time of each of the three stages
    float Tv, Ta, Td;

    // initialize class and destruct class
    trapezoid(const float v_max, const float a_max, const float d_max);
    ~trapezoid();

    // caluculate the trapezoidal profile
    // float noJerkProfile(float q0, float qe, float v0, float* trapVariables);
    void noJerkProfile(float q0, float qe, float v0);
    void buildArray();
};
