#include <Arduino.h>

// see algorithm in https://sam.ensam.eu/bitstream/handle/10985/11974/LSIS_CEP_2017_BEAREE.pdf?sequence=1&isAllowed=y

class jerkConsTraj {
  private:
    // acceleration limited profile initial conditions
    float q0;    // initial location
    float v0;    // adapted initial conditions
    // jerk limited profile initial conditions
    float q0f;    // initial location
    float v0f;    // initial velocity
    float a0f;    // initial acceleration
    // end location (position set point)
    float qe;
    // kinematic bounds
    float v_max, a_max, d_max, j_max;
    // total distance needed to travel
    float Dq;

    // calculate the sign of v0 for the trapezoidal profile function
    byte checkSign(float value);
    // calculate the trapezoidal profile times and reached values
    void trapProfile();
    // calculate the triangular profile times and reached values
    void triangProfile();

    void noZeroAccel();
    // void zeroAccel();

  public:
    // Total trap time
    float Tt;
    // duration time of each of the three stages for acceleration limited profile
    float Tv, Ta, Td;
    // reached values of velocity and acceleration
    float vr, ar, dr;
    // duration time of each of the added stages for jerk limited profile
    float Tj, Tja, Tjv, Tjd;

    // initialize class and destruct class
    jerkConsTraj(const float v_max, const float a_max, const float d_max, const float j_max);
    ~jerkConsTraj();

    // calculate the trapezoidal profile
    void noJerkProfile(float q0, float qe, float v0);
    void buildArray();

    // calculate the FIR filter profile
    void jerkProfile(float a0f);

};
