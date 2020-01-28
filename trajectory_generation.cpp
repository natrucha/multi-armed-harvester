#include <Arduino.h>
#include "trapezoid.h"

const float max_vel  = 1.;
const float max_acc  = 16.;
const float max_deac = 16.;

// see algorithm in https://sam.ensam.eu/bitstream/handle/10985/11974/LSIS_CEP_2017_BEAREE.pdf?sequence=1&isAllowed=y
trapezoid xAxis(max_vel, max_acc, max_deac);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // xAxis.noJerkProfile(5, 7, 1, &trapVariables[0]);
  xAxis.noJerkProfile(0, 0.1, 0.5);

  // Serial.print("reach velocity: ");
  // Serial.print(xAxis.vr);
  // Serial.print(" reach acceleration: ");
  // Serial.print(xAxis.ar);
  // Serial.print(" reach deacceleration: ");
  // Serial.println(xAxis.dr);
  //
  // Serial.print("Tv: ");
  // Serial.print(xAxis.Tv);
  // Serial.print(" Ta: ");
  // Serial.print(xAxis.Ta);
  // Serial.print(" Td: ");
  // Serial.println(xAxis.Td);
  // Serial.println();

  // xAxis.buildArray();

  xAxis.~trapezoid();
}
