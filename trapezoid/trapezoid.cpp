#include "trapezoid.h"

// direction (sign) of trajectory
int s;
// minimum stopping distance
float Dq_stop;
// minimum stopping velocity
float Dq_v0;
// min distance for which max velocity is reached
float Dq_v_max;
// number of points in the array
const int n = 150;
// Amount of time between time points in the array
float dT;
// Initialize arrays
float t[n];  // time
float v[n];  // velocity
float q[n];  // displacement (might use in closed loop control, otherwise remove to save sapce)


/* ****************************** */
/*       Class Initializer        */
/* ****************************** */
trapezoid::trapezoid(const float v_max, const float a_max, const float d_max){
  this->v_max = v_max;
  this->a_max = a_max;
  this->d_max = d_max;
}

/* ***************************** */
/*       Class Destructor        */
/* ***************************** */
trapezoid::~trapezoid(){ /* Exciting! */ }


/* ***************************** */
/* Determine the sign of a value */
/* ***************************** */
byte trapezoid::checkSign(float value) {
  byte sign = 1;
  if (value < 0) {
    sign = -1;
  }
  return sign;
}

/* ********************************************************** */
/* Calculate the trapezoidal profile times and reached values */
/* ********************************************************** */
void trapezoid::trapProfile() {
  ar = s * a_max;
  dr = s * d_max;
  vr = s * v_max;

  Ta = (vr - v0) / ar;
  Td = vr / dr;
  Tv = (Dq - s*Dq_v_max) / vr;

}

/* ********************************************************* */
/* Calculate the triangular profile times and reached values */
/* ********************************************************* */
void trapezoid::triangProfile() {
  ar = s * a_max;
  dr = s * d_max;
  vr = s * sqrt( (2*a_max*d_max*Dq - d_max*sq(v0)) / (a_max + d_max) );

  Ta = (vr - v0) / ar;
  Td = vr / dr;
  Tv = 0;
}

/* *********************************************************************** */
/* Calculate the acceleration limited profile (triangular or trapezoidal) */
/* *********************************************************************** */
void trapezoid::noJerkProfile(float q0, float qe, float v0) {
  this->q0 = q0;
  this->qe = qe;
  this->v0 = v0;
  this->Dq = Dq;

  byte sign;

  Dq_stop = sq(v0) / (2 * d_max);
  Dq = qe - q0;

  // check if a full stop trajectory is needed
  if (abs(Dq) <= Dq_stop) {
    sign = checkSign(v0);
    v0 = 0;
    q0 = q0 + sign*Dq_stop;
  }
  // special case found when v_max is changed on the fly (shouldn't happen...)
  if (abs(v0) > v_max) {
    sign = checkSign(v0);
    v0 = sign*v_max;
    Dq_v0 = sq(abs(v0) - v_max) / (2*d_max);
    q0 = q0 + sign*Dq_v0;
  }

  Dq_v_max = ( (sq(v_max)-sq(v0)) / (2*a_max) ) + (sq(v_max) / (2*d_max));

  s = checkSign(Dq);
  // check if the profile is trapezoidal or triangular
  if (abs(Dq) >= Dq_v_max) { // trapezoidal
    Serial.println("Trapezoidal");
    trapProfile();
  } else {                   // triangular
    triangProfile();
    Serial.println("Triangular");
  }
  Serial.println();

  this -> vr = vr;
  this -> ar = ar;
  this -> dr = dr;
  this -> Tv = Tv;
  this -> Ta = Ta;
  this -> Td = Td;
}

/* ********************************************************************* */
/* Build a X num points array based on the total time and reached values */
/* ********************************************************************* */
void trapezoid::buildArray() {
  this -> Tt = Tt;
  Tt = abs(Tv) + abs(Ta) + abs(Td); // calculate the total time for displacement

  dT = Tt / n;

  // initial values for the arrays
  t[0] = 0;
  v[0] = v0;
  q[0] = q0;

  // Serial.print(t[0], 4);
  // Serial.print(",");
  // Serial.print(v[0], 4);
  // Serial.print(",");
  // Serial.println(q[0], 4);

  // had issues adding q[i-1], so d serves as a stand in for q[i-1]
  float d = 0;

  // calculate the velocity at each time point
  for (int i = 1; i < n; i++) {
    t[i] = t[i-1] + dT;
    if ( t[i] <= abs(Ta) ) {
      v[i] = v0 + ar*t[i];

    } else if ( t[i] <= (abs(Ta) + abs(Tv)) ) {
      v[i] = vr;

    } else if ( t[i] <= (abs(Ta) + abs(Tv) + abs(Td)) ) {
      v[i] = vr - dr*(t[i] - (abs(Ta)+abs(Tv)));
    }

    // calculate the acceleration at each time point
    // a[i] = (v[i] - v[i-1]) / (t[i] - t[i-1]); // derivative of the velocity profile

    // calculate the displacement at each time point
    q[i] = v[i-1]*(t[i] - t[i-1]) + d;  // integrates the velocity
    d = q[i];

    // Serial.print(t[i], 4);
    // Serial.print(",");
    // Serial.print(v[i], 4);
    // Serial.print(",");
    // Serial.print(q[i], 4);
    // Serial.println();

  }

  // Serial.println();
  // Serial.println();
}
