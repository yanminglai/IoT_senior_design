#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define IMUdelay 10 // IMU refresh rate in milisec. smaller the delay, more accurate the measurements.

unsigned long previous_time = 0;                      //last cycle time
unsigned long current_time = 0;                          //current cycle time
unsigned long init_time = 0;                          // time when enter when enter loop

int waistspinmax = 0;
int waistspinmin = 0;
int modified_eulerx = 0;
int spincount =0;
int bending_angle_max = 0;
int hipanglemax = 0;
int bending_elbow = 0;
bool positionflag; //bending hand

Adafruit_BNO055 bno = Adafruit_BNO055();

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};


void setup() {
  Serial.begin(115200);

   if (!bno.begin())
  {
    Serial.printf("No BNO055 detected ...");  // Connection failed
    while (1);
  }
  delay(1000);


  bno.setExtCrystalUse(true);
  // Create the BLE Device
  BLEDevice::init("IoT movementsensor");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {

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

    //SerialBT.print(current_time - init_time);        // print current time stamp
    
    //recognize movement

    if(positionflag != 2){   //fullly

      //bending elbow
      //charcteristic invloved: euler.y() and gyroscope.y()

      if(euler.y() <= 10 & gyroscope.y() <= 5){ //detect init(flat) position
        positionflag = 1;
        bending_angle_max = 0;
      }

      bending_angle_max = _max(euler.y(), bending_angle_max);
      
      
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
      hipanglemax = waistspinmax - waistspinmin;
     
      if((waistspinmax - waistspinmin) > 180){
        spincount++;
        waistspinmax = 1;
        waistspinmin = 0;
        
      }

      }
      
      
    }
      /*
      SerialBT.print("bend=");
      SerialBT.print(bending_elbow);
      //SerialBT.print("; ");  //debug
      //SerialBT.print(positionflag);
      SerialBT.print("\t\t");
      SerialBT.print("spin=");
      SerialBT.print(spincount);
      SerialBT.println("\t");
      delay(200);
      */
     if (deviceConnected) {
    String datastring, slash;
    //datastring = euler.y() + ',' + accelerometer.y() + ',' +gyroscope.y() + ',' + temp;
    slash = String("/");
    datastring += euler.y();
    datastring += slash;
    datastring += bending_angle_max;
    datastring += slash;
    datastring += bending_elbow;
    datastring += slash;
    datastring += modified_eulerx;
    datastring += slash;
    datastring += hipanglemax;
    datastring += slash;
    datastring += spincount;
    

    
    //Serial.printf("*** Sent Value: %d ***\n", txValue);
    //Serial.printf("*** Sent Value: %d ***\n", euler.y());



    //DateTime now = rtc.now();
     //String str = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC) + " " + String(now.hour(), DEC) + ':' + String(now.minute(), DEC) + ':' + String(now.second(), DEC);
    //pCharacteristic->setValue(datastring.c_str());
    pCharacteristic->setValue(datastring.c_str());
    pCharacteristic->notify();
    txValue++;
  }
  //delay(1000);
      

    

    previous_time += IMUdelay;             // uptade previousTime
  }




  

}
