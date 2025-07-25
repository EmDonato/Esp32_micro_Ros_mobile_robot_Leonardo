
#include <PID.h>


float dt = 0.01;
float fake_rpm = 85.0;
// PID parameters: Kp, Ki, Kd, dt (s), integratorMin, integratorMax, outMin, outMax
PID pid(5.0, 2.0, 1.0,
        dt,   // 10 ms interval
        -500.0, 500.0,
        0,      255);




void setup() {
  Serial.begin(115200);
  // initialize PID state
  pid.reset();
}

void loop() {

  // compute PID
  int output = pid.compute(130.0, fake_rpm);

  // debug
  Serial.print("RPM: "); Serial.print(rpm);
  Serial.print("  PID: "); Serial.println(output);

  delay(dt*100);
}

