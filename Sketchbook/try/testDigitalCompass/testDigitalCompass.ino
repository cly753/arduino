#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
    
  error = compass.SetScale(0.88);
  if(error != 0)
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void loop() {
  MagnetometerRaw raw = compass.ReadRawAxis();
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  int MilliGauss_OnThe_XAxis = scaled.XAxis;

  float heading = atan2(raw.YAxis, raw.XAxis);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
   
  
  float headingDegrees = heading * 180/M_PI; // Convert radians to degrees for readability.

  Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  // delay(66);
  delay(1000);
}

void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees) {
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("   ");   
   Serial.print(raw.ZAxis);
   Serial.print("   \tScaled:\t");
   
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);

   Serial.print("   \tHeading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   Serial.print(headingDegrees);
   Serial.println(" Degrees   \t");
}
