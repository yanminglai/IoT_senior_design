

/*
 * IoT design, Yanming Lai
 * 
 * Supported movement: bending elbow and spin waist
 * */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BluetoothSerial.h>

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

BluetoothSerial SerialBT;  //enable bluetooth modual

//------------------------------------------------------------------------------------------------//
void setup(void)
{
  Serial.begin(9600);
  SerialBT.begin("ESP32");
  //SerialBT.println("The device started, now you can pair it with bluetooth!");
  //SerialBT.println("BNO055 Data Test");
  //SerialBT.println("");

  // Initialise the sensor
  if (!bno.begin())
  {
    SerialBT.print("No BNO055 detected ...");  // Connection failed
    while (1);
  }
  delay(1000);
  
  /*
  int8_t temp = bno.getTemp();   //get temp at first
  SerialBT.print("Current Temperature: ");
  SerialBT.print(temp);
  SerialBT.println(" C");
  SerialBT.println("");
*/
  bno.setExtCrystalUse(true);
  /*
  SerialBT.println("Calibration status: 0=uncalibrated, 3=fully calibrated");



  SerialBT.print("\t\t");
  SerialBT.print("Position");
  SerialBT.print("\t\t\t");
  SerialBT.print("Angular velocity");
  SerialBT.print("\t\t\t\t");
  SerialBT.print("Acceleration");
  SerialBT.print("\t\t\t\t");
  SerialBT.print("Calibration");
  SerialBT.print("\t\t");
  SerialBT.print("Bendcount/debugflag");
  SerialBT.print("\t\t");
  SerialBT.println("spincount/max/min"); //spin count
    


  SerialBT.print("Time ");
  SerialBT.print("\t\t");
  SerialBT.print("Roll; Pitch; Yaw");
  SerialBT.print("\t\t");
  SerialBT.print("Roll rate; Pitch rate; Yaw rate");
  SerialBT.print(" \t\t");
  SerialBT.print("Accel X, Accel Y, Accel Z");
  SerialBT.print("\t\t");
  SerialBT.println("Sys; Gyro; Accel; Mag");
  */
  SerialBT.flush();             // wait for header line to go out, used to be serial.flush
  
  init_time = millis();         // get time and set as init
  previous_time = init_time;   //prepare
  
}


//------------------------------------------------------------------------------------------------//
void loop(void)
{
  //SerialBT.print("Hello World");
  
  
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

    /*SerialBT.print(current_time - init_time);        // print current time stamp
    //SerialBT.print("\t\t");
    
    SerialBT.print(euler.x());
    SerialBT.print("; ");
    SerialBT.print(euler.y());
    SerialBT.print("; ");
    SerialBT.print(euler.z());
    SerialBT.print("\t\t");

    SerialBT.print(gyroscope.x());
    SerialBT.print("; ");
    SerialBT.print(gyroscope.y());
    SerialBT.print("; ");
    SerialBT.print(gyroscope.z());
    SerialBT.print(" \t\t\t\t");

    SerialBT.print(accelerometer.x());
    SerialBT.print("; ");
    SerialBT.print(accelerometer.y());
    SerialBT.print("; ");
    SerialBT.print(accelerometer.z());
    SerialBT.print("\t\t\t");

    SerialBT.print(system, DEC);  //for calibration 
    SerialBT.print("; ");
    SerialBT.print(gyro, DEC);
    SerialBT.print("; ");
    SerialBT.print(accel, DEC);
    SerialBT.print("; ");
    SerialBT.print(mag, DEC);
    SerialBT.print("\t\t");

    */
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
      waistspinmax = _max(waistspinmax, modified_eulerx); //keep updating max euler.x()
      waistspinmin = _min(waistspinmin, modified_eulerx);

     
      if((waistspinmax - waistspinmin) > 180){
        spincount++;
        waistspinmax = 1;
        waistspinmin = 0;
        
      }

      }
      
      
    }

      //SerialBT.print("\t");
      SerialBT.print(bending_elbow);
      //SerialBT.print("; ");  //debug
      //SerialBT.print(positionflag);
      SerialBT.print("\t\t");
      SerialBT.print(spincount);
      SerialBT.println("\t");
      delay(200);
      //SerialBT.print("; ");
      //SerialBT.print(waistspinmax);
      //SerialBT.print(";");
      //SerialBT.println(waistspinmin);
      
      
      

    

    previous_time += IMUdelay;             // uptade previousTime
  }

}
