/*
 * IoT design, Yanming Lai
 * 
 * Supported movement: bending elbow and spin waist
 * */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//initializations
unsigned long previous_time = 0;                      //last cycle time
unsigned long current_time = 0;                          //current cycle time
unsigned long init_time = 0;                          // time when enter when enter loop

int waistspinmax = 0;
int waistspinmin = 0;
int modified_eulerx = 0;
int spincount =0;


int bending_elbow = 0;
bool positionflag;

#define IMUdelay 10 // IMU refresh rate in milisec. smaller the delay, more accurate the measurements.

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
  Serial.print("Bendcount/debugflag");
  Serial.print("\t\t");
  Serial.println("spincount/max/min"); //spin count
    


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

      //bending elbow
      //charcteristic invloved: euler.y() and gyroscope.y()

      if(euler.y() <= 10 & gyroscope.y() <= 5){ //detect init(flat) position
        positionflag = 1;
      }

      if(positionflag == 1 & euler.y() >= 70 & gyroscope.y() <= 5){ //destination position
        
        bending_elbow += 1;
        positionflag = 0;
        
      }

      //spinning waist

      //charcteristic invloved: euler angle and accelerometer

      //find max and min of euler.x() if max-min >= 170 degree 

      if(gyroscope.x() < 10){  //only take about the data when moving is about to end
                               //to prevent double counting
        
      if(euler.x() > 180){
        modified_eulerx = euler.x() - 360;
      }
      else{
      modified_eulerx = euler.x();  //modified value of eulerx from (0,360) to (-180,180)
      }
      waistspinmax = max(waistspinmax, modified_eulerx); //keep updating max euler.x()
      waistspinmin = min(waistspinmin, modified_eulerx);

     
      if((waistspinmax - waistspinmin) > 150){
        spincount++;
        waistspinmax = 1;
        waistspinmin = 0;
        
      }

      }
      
      
    }

      Serial.print("\t");
      Serial.print(bending_elbow);
      Serial.print("; ");  //debug
      Serial.print(positionflag);
      Serial.print("\t\t");
      Serial.print(spincount);
      Serial.print("; ");
      Serial.print(waistspinmax);
      Serial.print(";");
      Serial.println(waistspinmin);
      
      
      

    

    previous_time += IMUdelay;             // uptade previousTime
  }

}
