#define rd 562.0
#define oneGrid 281.0
#define one360 1650.0 // hall stair
#define one360 1633.0 // hall room floor

________________________________________
************update parameter************
________________________________________

float getDis21(int pin) {
  return 12343.85 * pow(analogRead(pin),-1.15);
}

void PWM_Mode_Setup() {
  pinMode(urTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(urTRIG,HIGH);                  // Set to HIGH
  pinMode(urPWM, INPUT);                      // Sending Enable PWM mode command
  uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};
  for(int i=0;i<4;i++)
    Serial.write(EnPwmCmd[i]);
}
int PWM_Mode_getDis() { // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(urTRIG, LOW);
    digitalWrite(urTRIG, HIGH);               // reading Pin PWM will output pulses
    return pulseIn(urPWM, LOW) / 50;
}

void setCompass() {
  compass = HMC5883L();
  compass.SetMeasurementMode(Measurement_Continuous);
}
float getHeading() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  if (heading < 0)
    heading += 2 * PI;
  return heading * 180.0 / M_PI;
}

void goAhead(float grid) {
    int need = grid * oneGrid;

    if (need < 0) {
        need *= -1;
        md.setSpeeds(-200, -200);
    } else {
        md.setSpeeds(200, 200);
    }

    while (need--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }

    md.setBrakes(400, 400);
}
void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
    int need = degree / 360.0 * one360;

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(-200, 200);
    } else {
        md.setSpeeds(200, -200);
    }

    while (need--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }

    md.setBrakes(400, 400);
}
void shiftLeft(float grid) {
    rotateLeft(45);
    goAhead(1.414 * grid);
    rotateLeft(-45);
    goAhead(-1.414 * grid);
}
void selfAdjust(float leftHead, float leftTail) {
    if (14 < leftHead && 14 < leftTail) {

    } else if (10 < leftHead && leftHead < 14 && 10 < leftTail && leftTail < 14) {
        shiftLeft(1.5);
    } else if (7 < leftHead && leftHead < 10 && 7 < leftTail && leftTail < 10) {
        shiftLeft(1);
    } else if (leftHead < 4 && leftTail < 4) {
        shiftLeft(-1);
    } else if (4 < leftHead && leftHead < 7 && 7 < leftTail) {
        rotateLeft(-10);
    } else if (7 < leftHead && 4 < leftTail && leftTail < 7) {
        rotateLeft(10);
    }
}
void adjustSpeed(int spe) {
    int left = 0;
    int right = 0;
    int leftTime;
    int rightTime;
    int count = 200;

    leftTime = millis();
    while (count--) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
    }
    leftTime = millis() - leftTime;

    count = 200;

    rightTime = millis();
    while (count--) {
        while (digitalRead(enRight));
        while (!digitalRead(enRight));
    }
    rightTime = millis() - rightTime;

    md.setSpeeds(spe - (leftTime - rightTime) * 0.1, spe + (leftTime - rightTime) * 0.1);
}

// void adjustHeading() {
//     int st = getHeading();
//     int error;

//     // go...go...

//     error = getHeading() - st;
//     rotateLeft(error);
// }

// compensate for brake

// speed(encoder feedback)
// keepStraight(encoder feedback)

// sense twice and wait enough time for stable data

// raise the direction of ultrasonic if placed vertical