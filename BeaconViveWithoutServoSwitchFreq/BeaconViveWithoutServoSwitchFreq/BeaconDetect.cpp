/*
 * Class for beacon detection
 */

#include "BeaconDetect.h"

// a function used to change the read in pin state to an int
int BeaconDetect::mydigitalRead(int pin){
  if(digitalRead(pin) == HIGH){
    return 1;
  }
  return 0;
}


// a function used to check the signal frequency recieved from the given channel
void BeaconDetect::checkFreq(int ch, boolean delayFlag) {
  static int oldpin[2];  // a int array to record input pin state
  static uint32_t oldtime[2];  // a int array to record the last state showed up time for each pin
  static int count[2];  // a int array to record the change times of the pin state
  static int isSense[2]; // a int array to record the sensing state of the phototransistor

  if (mydigitalRead(pin[ch]) != oldpin[ch]) {  // if the pin state has changed
    count[ch] = count[ch] + 1; // record one time of pin state change
    oldpin[ch] = mydigitalRead(pin[ch]);  // record the new pin state
  }
  if (( count[ch] > averageCount)||(delayFlag)){  // if the state of the pin have already changed 10 times or the delay flag had been set to true
    isSense[ch] = 0; 
    if(delayFlag){
      oldtime[ch] = us;  // re cord the new state occurd time
      count[ch] = 0;  // reset the count time to 0
      oldpin[ch] = mydigitalRead(pin[ch]); 
    }
    else{
      int per = (us-oldtime[ch])/((averageCount+1)/2);  // calculate the time period between of one cycle
      oldtime[ch] = us;  // re cord the new state occurd time
//      Serial.print(ch);
//      Serial.print(":");
//      Serial.println(1000000.0/per);
      count[ch] = 0;  // reset the count time to 0
      if (per>period-noise && per<period+noise) {  // if the time period is in certain range
          if (ch==0){
            isSense[ch] = 1;
          }
          else{ // else, servo derection truned to channel no.1 derection
            isSense[ch] = 2;  
          }
      }
      isSensing = isSense[0] + isSense[1];
    }
  }
}


// a function to initialize the BeaconDetect class
BeaconDetect::BeaconDetect(int Pin1, int Pin2, int DetectFrequency){
  pin1 = Pin1;
  pin2 = Pin2;
  detectFrequency = DetectFrequency;
  
  pin[0] = pin1;
  pin[1] = pin2;
  period = 1000000.0/(detectFrequency);
  noise = 0.1 * period; 
  averageCount = floor(detectFrequency/80) + 1;
}


// a function used to change the detect frquency
void BeaconDetect::changeFreq(int frequency){
  detectFrequency = frequency;
  period = 1000000.0/(detectFrequency);
  noise = 0.1 * period; 
  averageCount = floor(detectFrequency/80) + 1;  
}

// a function to used to setup beacon detection
void BeaconDetect::beaconSetup() {
  pinMode(pin1,INPUT);
  pinMode(pin2,INPUT);
}

// a function to control 
void BeaconDetect::control(boolean delayFlag) {
  us = micros();  // record the programm processing time
  checkFreq(0,delayFlag);  // check the input frequency of channel no.0
  checkFreq(1,delayFlag);  // check the input frequency of channel no.1
}
