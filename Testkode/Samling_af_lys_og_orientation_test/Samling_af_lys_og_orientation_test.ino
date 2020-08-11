#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN        5 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 6 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

int orangeState = 0;
int blaaState = 0;


/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) {

  // Kør funktionerne i loop - lav funktioner til når de finder værdierne og eksekver dererfterfunktionerne


  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data - JEG VED IKKE LIGE HVOR DE HER FUNKTIONER LIGGER? EVT I BIBLIOTEKET? */
  Serial.print("X: ");
  Serial.print(euler.x());
   Serial.print(" Y: ");
    Serial.print(euler.y());

    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t"); 



  /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);


  if (euler.x() >= 300 && orangeState == 0) {
    orange();
  }

  if (euler.y() >= 30 && euler.x() <= 299 && blaaState == 0) {
    blaa();
  }
}

void orange() {
  Serial.println("orange");


  // det her virker tydligvis, lav lignende
  // lav en for stationær, stablet, skråt (se om der er nogle ligheder.)
  for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(240, 100, 50));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
    orangeState = 1;
    blaaState = 0;

  }
}

void blaa() {

  // det her virker tydligvis, lav lignende
  // lav en for stationær, stablet, skråt (se om der er nogle ligheder.)
  for (int i = 0; i < NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(240, 200, 150));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
    orangeState = 0;
    blaaState = 1;



}
}
