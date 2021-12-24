
#include "BeaconDetect.h"
#include "vive510.h"


#define PIN1 32 // signal input pin1
#define PIN2 33 // signal input pin2
#define DETECTFREQUENCY  23  // the frequency tried to detect
#define SIGNALPIN1 14 // pin receiving signal from Vive circuit
#define SIGNALPIN2 27 // pin receiving signal from Vive circuit


//BeaconDetect beaconDetect(PIN1,PIN2,DETECTFREQUENCY);
BeaconDetect beaconDetect(PIN1,PIN2,DETECTFREQUENCY);
Vive510 viveGroup7(SIGNALPIN1);
Vive510 viveGroup7_2(SIGNALPIN2);
static uint32_t lastDetectTime;  // record the last time when detect beacon

boolean delayFlag = false;  // a flag to show whether the beacon program has been delayed

void setup() {
  Serial.begin(115200);

  // set up the beacon and vive program
  beaconDetect.beaconSetup();
  viveGroup7.begin();
  viveGroup7_2.begin();
}

void loop() {
  
  int x_1;
  int y_1;
  int x_2;
  int y_2;

  // listen to raspberry pi and change the detect frequency accordingly
  if (Serial.available() >= 3) {
    char buf[3];
    for( int i=0; i<3; i++){
      buf[i] = Serial.read();
    }
    int detectFreq = atoi(buf);   
    beaconDetect.changeFreq(detectFreq);  
    delayFlag = true;
    lastDetectTime = micros();  
  }
  
  // if the beacon detect program haven't run 0.5s
  if (micros()-lastDetectTime < 100000){
    // keep running the beacon detect program
    beaconDetect.control(delayFlag);
    delayFlag = false;
  }
  else{
    // if the beacon detect program have already run for 0.5s
    
    // start the vive detect program    
    if (viveGroup7.status() == VIVE_LOCKEDON) {
      // read in the x,y value from the first vive circuit
      x_1 = viveGroup7.xCoord();
      y_1 = viveGroup7.yCoord();
      // check whether the read in value is in the correct range 
      if(( x_1>9000) || (x_1<2000) || (y_1>9000) || (y_1<2000)){
        x_1 = 0;
        y_1 = 0;
      }
    }
    else{ 
      viveGroup7.sync(15); // try to resync (nonblocking);
      x_1 = 0;
      y_1 = 0;
    }
    
    if (viveGroup7_2.status() == VIVE_LOCKEDON) {
      // read in the x,y value from the second vive circuit
      x_2 = viveGroup7_2.xCoord();
      y_2 = viveGroup7_2.yCoord();
      // check whether the read in value is in the correct range 
      if( (x_2>9000)|| (x_2<2000) || (y_2>9000) || (y_2<2000)){
        x_2 = 0;
        y_2 = 0;
      }     
    }
    else{ 
      viveGroup7_2.sync(15); // try to resync (nonblocking);
      x_2 = 0;
      y_2 = 0;
    }

    // set the beacon detect delay flag to true, indicate that the beacon detect program had stop running for a while
    delayFlag = true;
    Serial.printf("Vive,%d,%d,%d,%d \n",x_1,y_1,x_2,y_2);
    Serial.printf("Beacon,%d \n",beaconDetect.isSensing);
//    Serial.println(beaconDetect.averageCount);
    lastDetectTime = micros();
  } 
}
