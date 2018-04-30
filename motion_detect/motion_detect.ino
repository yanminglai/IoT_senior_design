#include <Wire.h>
/*
 * IoT design, Yanming Lai
 * */
 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//initializations

unsigned long previous_time = 0;                      //last cycle time
unsigned long current_time = 0;                          //current cycle time
unsigned long init_time = 0;                          // time when enter when enter loop

int bending_wrist = 0;
bool positionflag;

#define IMUdelay 200 // IMU refresh rate in milisec was 1000

Adafruit_BNO055 bno = Adafruit_BNO055();

//------------------------------------------------------------------------------------------------//
void setup(void)
{
  Serial.begin(9600);
  Serial.println("BNO055 Data Test");
  Serial.println("");

  // Initialise the sensor
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected ...");  // Connection failed
    while (1);
  }
  delay(1000);
  
  
  int8_t temp = bno.getTemp();   //get temp at first
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status: 0=uncalibrated, 3=fully calibrated");



  Serial.print("\t\t");
  Serial.print("Position");
  Serial.print("\t\t\t");
  Serial.print("Angular velocity");
  Serial.print("\t\t\t\t");
  Serial.print("Acceleration");
  Serial.print("\t\t\t\t");
  Serial.print("Calibration");
  Serial.print("\t\t");
  Serial.println("Bendcount/debugflag");
    


  Serial.print("Time ");
  Serial.print("\t\t");
  Serial.print("Roll; Pitch; Yaw");
  Serial.print("\t\t");
  Serial.print("Roll rate; Pitch rate; Yaw rate");
  Serial.print(" \t\t");
  Serial.print("Accel X, Accel Y, Accel Z");
  Serial.print("\t\t");
  Serial.println("Sys; Gyro; Accel; Mag");

  Serial.flush();             // wait for header line to go out
  
  init_time = millis();         // get time and set as init
  previous_time = init_time;   //prepare
  
}


//------------------------------------------------------------------------------------------------//
void loop(void)
{
  current_time = millis();

  if (current_time - previous_time >= IMUdelay) { //enter next loop
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    //https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);   // get calibration

    Serial.print(current_time - init_time);        // print current time stamp
    Serial.print("\t\t");

    Serial.print(euler.x());
    Serial.print("; ");
    Serial.print(euler.y());
    Serial.print("; ");
    Serial.print(euler.z());
    Serial.print("\t\t");

    Serial.print(gyroscope.x());
    Serial.print("; ");
    Serial.print(gyroscope.y());
    Serial.print("; ");
    Serial.print(gyroscope.z());
    Serial.print(" \t\t\t\t");

    Serial.print(accelerometer.x());
    Serial.print("; ");
    Serial.print(accelerometer.y());
    Serial.print("; ");
    Serial.print(accelerometer.z());
    Serial.print("\t\t\t");

    Serial.print(system, DEC);  //for calibration 
    Serial.print("; ");
    Serial.print(gyro, DEC);
    Serial.print("; ");
    Serial.print(accel, DEC);
    Serial.print("; ");
    Serial.print(mag, DEC);
    Serial.print("\t\t");
    //recognize movement

    if(positionflag != 2){   //fullly

      //bending wrist
      //charcteristic invloved: euler.y() and gyroscope.y()

      if(euler.y() <= 10 & gyroscope.y() <= 5){ //detect init(flat) position
        positionflag = 1;
      }

      if(positionflag == 1 & euler.y() >= 70 & gyroscope.y() <= 5){ //destination position
        
        bending_wrist += 1;
        positionflag = 0;
        
      }

           

      
      
      
    }

      Serial.print("\t");
      Serial.print(bending_wrist);
      Serial.print("; ");  //debug
      Serial.println(positionflag);
      
      

    

    previous_time += IMUdelay;             // uptade previousTime
  }

}
