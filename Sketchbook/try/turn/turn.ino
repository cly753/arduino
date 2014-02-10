// turn.ino
#include <Wire.h>
#include <HMC5883L.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>

double input, output, target;
double kp = 15, ki = 0, kd = 0;

HMC5883L compass;
PID pid(&input, &output, &target, kp, ki, kd, DIRECT);
DualVNH5019MotorShield md;
MagnetometerScaled scaled;
double heading;

void setPID() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-200, 200);
}

void setCompass() {
    compass = HMC5883L();
    compass.SetMeasurementMode(Measurement_Continuous);
}
double getHeading() {
    scaled = compass.ReadScaledAxis();
    heading = atan2(scaled.YAxis, scaled.XAxis);
    if (heading < 0)
        heading += 2 * PI;
    return heading * 180 / PI;
}

void setMotor() {
    md.init();
    md.setBrakes(400, 400);
}
void turn(int degree) {
    double init = getHeading();
    Serial.print("\n\n\ninit: "); 
    Serial.println(init);   

    target = getHeading() + degree;

    Serial.print("target: "); 
    Serial.println(target);

    delay(2000);

    //target = 250;

    while (1) {
        input = getHeading();

        Serial.print("\ntarget: "); 
        Serial.println(target);
        Serial.print("input: "); 
        Serial.println(input);


        pid.Compute();
        md.setSpeeds(-1 * output, output);

        Serial.print("output: "); 
        Serial.println(output);

        delay(100);
    }

    // for (int i = 0; i < 50; i++) {
    //     input = getHeading();
    //     pid.Compute();
    //     md.setSpeeds(output, -1 * output);

    //     delay(66);
    // }

    // md.setBrakes(400, 400);
    // while (1) {

    // }
}


void setup() {
    delay(1000);
    Wire.begin();
    Serial.begin(9600);
    setMotor();
    setCompass();
    setPID();

    turn(90);
}

void loop() {

}