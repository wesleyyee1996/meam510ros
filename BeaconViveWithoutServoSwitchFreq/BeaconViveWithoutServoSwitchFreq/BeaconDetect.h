/*
 * header for beacon detection 
 */

#ifndef BEACONDETECT
#define BEACONDETECT

#include<math.h>
#include <arduino.h>



class BeaconDetect
{
private:
  /*
   * the using pins
   */
  int pin1;  // phototransistor signal input pin1
  int pin2;  // phototransistor signal input pin2
  int pin[2];  // a int array to record the input pin

  /*
   * frequency to be detected related params
   */
  int detectFrequency; // the frequency tried to detect
  double period;  // the detect signal time period (1000000/(frequency))
  double noise;  // the reasonable frequency detect error
  uint32_t us;  // a variable to record program processing time
  boolean delayFlag = false;


  /*
   * methods
   */
  int mydigitalRead(int pin);  // a function used to change the read in pin state to an int
  void checkFreq(int ch, boolean delayFlag);  // a function used to check the signal frequency recieved from the given channel

  
public:
  int isSensing; // a variable to indicate whether any of the phototransistor is reciving a signal
  int averageCount;  // the mount to count before taking the average
  BeaconDetect(int Pin1, int Pin2, int DetectFrequency);
  void beaconSetup();  // a function to used to setup beacon detection
  void control(boolean delayFlag);  // a function to control 
  void changeFreq(int frequency); // a function to change the detect frequency
};


#endif
