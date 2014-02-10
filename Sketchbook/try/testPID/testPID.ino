// testPID.ino
#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>

DualVNH5019MotorShield md;
HMC5883L compass;
MagnetometerScaled scaled;

double input, output, target;
PID pid(&input, &output, &target, 2, 0, 1, DIRECT);
int spe;

int URPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG=5; // PWM trigger pin
int smooth[3] = {0, 0, 0};
int smoothSum = 0;

// void setCompass() {
//   int error = 0;
//   compass = HMC5883L();
//   error = compass.SetScale(1.3);
//   if (error != 0)
//     Serial.println(compass.GetErrorText(error));
//   error = compass.SetMeasurementMode(Measurement_Continuous);
//   if (error != 0)
//     Serial.println(compass.GetErrorText(error));
// }

// float getHeading() {
//   scaled = compass.ReadScaledAxis();
//   float heading = atan2(scaled.YAxis, scaled.XAxis);
//   if (heading < 0)
//     heading += 2 * PI;

//   return heading * 180 / M_PI;
// }

void PWM_Mode_Setup() {
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
  
  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
  
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
      Serial.write(EnPwmCmd[i]);
}

int PWM_Mode_getDis() {                              // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses
     
    return pulseIn(URPWM, LOW) / 50;
}

float smoothOutput(float output) {
  smoothSum += output;
  smoothSum -= smooth[2];
  for (int i = 0; i < 2; i++)
    smooth[i+1] = smooth[i];
  return smoothSum / 3.0;
}

void setPID() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-10, 10);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();

    setCompass();
    setPID();
    md.init();

    delay(100);

    target = getHeading();
    input = getHeading();

    Serial.print("input: ");
    Serial.println(input);
    Serial.print("target: ");
    Serial.println(target);

    delay(2000);

    spe = 200;
    md.setSpeeds(spe, spe);

    while (PWM_Mode_getDis() > 10) {
        input = getHeading();
        Serial.print("\nheading: ");
        Serial.println(input);

        pid.Compute();

        Serial.print("output: ");
        Serial.println(output);

        md.setSpeeds(spe + (int)smoothOutput(output), spe - (int)smoothOutput(output));

        delay(200);
    }

    md.setBrakes(400, 400);
}

void loop() {

}

