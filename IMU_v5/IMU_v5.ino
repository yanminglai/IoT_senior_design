#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NeoHWSerial.h>                                // fatser than native serial communication

// serial communication:
#define DEBUG_PORT_TYPE NeoHWSerial
#define DEBUG_PORT NeoSerial

// variables:
const int periodIMU = 10000;                            // IMU period in micros
unsigned long previousTimeIMU = 0;                      // end time of the last sensor period in micros
unsigned long currentTime = 0;                          // actual time in micros
unsigned long initialTime = 0;                          // time when main loop begin for the first time in micros

#define BNO055_SAMPLERATE_DELAY_MS (periodIMU/1000)     // IMU sample rate initialisation in ms

Adafruit_BNO055 bno = Adafruit_BNO055();

//------------------------------------------------------------------------------------------------//
void setup(void)
{
  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println("BNO055 Data Test");
  DEBUG_PORT.println("");

  // Initialise the sensor
  if (!bno.begin())
  {
    DEBUG_PORT.print("No BNO055 detected ... Check your wiring or I2C ADDR!");  // There was a problem detecting the BNO055 ... check your connections
    while (1);
  }
  delay(1000);
  
  // Display current temperature
  int8_t temp = bno.getTemp();   
  DEBUG_PORT.print("Current Temperature: ");
  DEBUG_PORT.print(temp);
  DEBUG_PORT.println(" C");
  DEBUG_PORT.println("");

  bno.setExtCrystalUse(true);

  DEBUG_PORT.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  // Display column headers
  DEBUG_PORT.println( F(
    "\t\t"  
    "Position "
    "\t\t\t"
    "Angular velocity"
    "\t\t\t\t"
    "Acceleration"
    "\t\t\t\t"
    "Calibration"));
    
  // Display column sub-headers
  DEBUG_PORT.println( F(
    "Time "
    "\t\t"
    "Roll; Pitch; Yaw"
    "\t\t"
    "Roll rate; Pitch rate; Yaw rate"
    " \t\t"
    "Accel X, Accel Y, Accel Z"
    "\t\t"
    "Sys; Gyro; Accel; Mag") );

  DEBUG_PORT.flush();             // wait for header line to go out
  
  initialTime = micros();         // start timing from NOW
  previousTimeIMU = initialTime;
  
}


//------------------------------------------------------------------------------------------------//
void loop(void)
{
  currentTime = micros();

  if (currentTime - previousTimeIMU >= periodIMU) {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);   // calibration statut

    DEBUG_PORT.print(currentTime - initialTime);        // current time at the begining of each loop
    DEBUG_PORT.print("\t\t");

    DEBUG_PORT.print(euler.x());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(euler.y());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(euler.z());
    DEBUG_PORT.print("\t\t");

    DEBUG_PORT.print(gyroscope.x());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(gyroscope.y());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(gyroscope.z());
    DEBUG_PORT.print(" \t\t\t\t");

    DEBUG_PORT.print(accelerometer.x());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(accelerometer.y());
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(accelerometer.z());
    DEBUG_PORT.print("\t\t\t");

    DEBUG_PORT.print(system, DEC);
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(gyro, DEC);
    DEBUG_PORT.print("; ");
    DEBUG_PORT.print(accel, DEC);
    DEBUG_PORT.print("; ");
    DEBUG_PORT.println(mag, DEC);

    previousTimeIMU += periodIMU;             // uptade previousTime
  }

}
