#include <Arduino.h>
#include "jerkConsTraj.h"

const float max_vel  = 1.;
const float max_acc  = 16.;
const float max_deac = 16.;
const float max_jerk = 250.;

float a0f;

// see algorithm in https://sam.ensam.eu/bitstream/handle/10985/11974/LSIS_CEP_2017_BEAREE.pdf?sequence=1&isAllowed=y
jerkConsTraj zAxis(max_vel, max_acc, max_deac, max_jerk);

void setup() {
  Serial.begin(9600);
}

void loop() {
  zAxis.noJerkProfile(0, 0.1, 0.5);

  // Serial.print("Total trapezoidal travel time: ");
  // Serial.println(xAxis.Tt);
  // Serial.println();

  // Serial.print("ar: ");
  // Serial.print(zAxis.ar, 4);
  // Serial.print(" vr: ");
  // Serial.print(zAxis.vr, 4);
  // Serial.print(" dr: ");
  // Serial.print(zAxis.dr, 4);
  // Serial.println();
  //
  // Serial.print("Ta: ");
  // Serial.print(zAxis.Ta, 4);
  // Serial.print(" Tv: ");
  // Serial.print(zAxis.Tv, 4);
  // Serial.print(" Td: ");
  // Serial.print(zAxis.Td, 4);
  // Serial.println();
  // Serial.println();

  // xAxis.buildArray();
  // will need to figure out how to calculate a0f after profile works
  a0f = 0;
  zAxis.jerkProfile(a0f); // not very nice is it?

  Serial.print("Tja: ");
  Serial.print(zAxis.Tja, 4);
  Serial.print(" Tjv: ");
  Serial.print(zAxis.Tjv, 4);
  Serial.print(" Tjd: ");
  Serial.print(zAxis.Tjd, 4);
  Serial.println();

  Serial.print("ar: ");
  Serial.print(zAxis.ar, 4);
  Serial.print(" vr: ");
  Serial.print(zAxis.vr, 4);
  Serial.print(" dr: ");
  Serial.print(zAxis.dr, 4);
  Serial.println();
  Serial.println();
  Serial.println();

  zAxis.~jerkConsTraj();
}
