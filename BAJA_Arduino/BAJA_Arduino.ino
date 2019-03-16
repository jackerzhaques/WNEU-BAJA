/*
 * Contributed Libraries
 * Use "Manage Libraries" menu option to download these
 */
#include <FreqMeasure.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

/*
 * Software:  BAJA Realtime Monitor
 * Author:    Todd Morehouse
 * Date:      2/13/2019
 * Description:
 *  This software monitors the RPM of the engine, as well as
 *  the orientation of the board. The RPM is a pulse train input,
 *  the frequency is monitored through an input capture pin. The
 *  frequency is output as an analog voltage through a DAC
 *  connected over I2C. The orientation of the board (and
 *  therefor the car) is monitored using a 6-axis accelorometer.
 *  
 */

/*
 * Debugging defines
 * 
 * Commenting out the define will allow debugging messages over serial
 */
 #define DEBUGGING_SETUP
 #define DEBUGGING_DAC
 #define DEBUGGING_RPM

 

 #include <Wire.h>

 #define  DAC_I2C_ADDRESS         0x62
 #define  DAC_MIN                 0.00
 #define  DAC_MAX                 5.00

 #define  RPM_INPUT_PIN           13
 #define  RPM_SAMPLING_AVERAGE    2
 #define  RPM_MIN                 10
 #define  RPM_MAX                 50

 Adafruit_BNO055 IMU = Adafruit_BNO055(55);

 //Forward declarations
 void SetFrequency(float Frequency);

 //DAC Device
 Adafruit_MCP4725 DAC;

 bool RPMMeasurementReceived = false;

 //Called when an rpm measurement hasn't come in after one second
 //Returns the output to 0
 void RPMMeasurementTimeout(){
  if(!RPMMeasurementReceived){
    SetFrequency(0);
  }
  else{
    RPMMeasurementReceived = false;
  }
 }

 //Resets the the RPM timeout
 void ResetRPMTimeout(){
  RPMMeasurementReceived = true;
 }
 

/*
 * Program entry
 */
void setup() {
  Serial.begin(115200);

  //The arduino micro needs a delay before using serial
  delay(1000);
  
  /*
   * Configure the FreqMeasure library to measure the frequency
   * of the RPM sensor input
   */
   #ifdef DEBUGGING_SETUP
    Serial.print("Initializing FreqMeasure on pin ");
    Serial.println(RPM_INPUT_PIN);
   #endif
   FreqMeasure.begin();
   pinMode(13, INPUT_PULLUP); //THIS HAS TO BE A NUMBER. NO VARIABLES/DEFINES. MESSES THIS UP FOR SOME REASON

   /*
    * Configure Wire (I2C) for the DAC
    */
   #ifdef DEBUGGING_SETUP
    Serial.println("Initializing Wire for the DAC");
   #endif
   DAC.begin(DAC_I2C_ADDRESS);

   //Default the DAC voltage to 0
   DAC.setVoltage(0, false);

   //Initialize the IMU
   if(!IMU.begin()){
    Serial.println("Failed to connect to the IMU!");
   }

   delay(1000);
   IMU.setExtCrystalUse(true);
   
   #ifdef DEBUGGING_SETUP
    Serial.println("Initialization done");
   #endif
}

float CurrentFrequency = 0;
float DACVoltage = 0;

float DAC_FrequencyToVoltage(float Frequency){
  float RPM_Range = RPM_MAX - RPM_MIN;
  float DAC_Range = DAC_MAX - DAC_MIN;
  float RPM_Normalized = (Frequency - RPM_MIN) / RPM_Range;
  float Voltage = (RPM_Normalized * DAC_Range) + DAC_MIN;
  return Voltage;
}

uint16_t DAC_VoltageToValue(float Voltage){
  float ratio = 4095/5.0;
  float Value = Voltage * ratio;
  return (uint16_t)(Value);
}

void UpdateDAC(){
  DACVoltage = DAC_FrequencyToVoltage(CurrentFrequency);
  DAC.setVoltage(DAC_VoltageToValue(DACVoltage), false);
}

void SetFrequency(float Frequency){
  //Clamp the frequency at the max and min RPMs
  if(Frequency < RPM_MIN){
    Frequency = RPM_MIN;
  }
  else if(Frequency > RPM_MAX){
    Frequency = RPM_MAX;
  }

  //Record the frequency
  CurrentFrequency = Frequency;
  
  //Update the DAC's output
  UpdateDAC();

  #ifdef DEBUGGING_RPM
    Serial.print("Frequency set to ");
    Serial.println(Frequency);
  #endif
}

//Returns true if the measurement changed
void GetFreqMeasurement(){
  static double FreqSum = 0;
  static int FreqSample = 0;
  
  if(FreqMeasure.available()){
    //Restart the RPM timeout
    ResetRPMTimeout();
    
    //Capture the frequency
    FreqSum += FreqMeasure.read();
    
    FreqSample++;
    if(FreqSample > RPM_SAMPLING_AVERAGE){
      SetFrequency(FreqMeasure.countToFrequency(FreqSum / FreqSample));
      FreqSum = 0;
      FreqSample = 0;
    }
  }
}

void ReadDOFAndPrint(){
  sensors_event_t event;
  IMU.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  sensors_event_t AccEvent;
  IMU.getEvent(&AccEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("\taX: ");
  Serial.print(AccEvent.acceleration.x, 4);
  Serial.print("\taY: ");
  Serial.print(AccEvent.acceleration.y, 4);
  Serial.print("\taZ: ");
  Serial.println(AccEvent.acceleration.z, 4);
}

typedef struct sTask_tag{
  uint32_t Count;
  uint32_t MaxCount; //Period of task in milliseconds
  void (*CallbackFcn)(void);
} sTask;

sTask TaskList[] = {
  //DAC Update
  {
    0,
    1,
    &GetFreqMeasurement
  },
  //DAC Timeout
  {
    0,
    1000,
    &RPMMeasurementTimeout
  },
  //DOF Read
  {
    0,
    10,
    &ReadDOFAndPrint
  }
};

uint8_t nTasks = sizeof(TaskList)/sizeof(sTask);

/*
 * Program main loop
 */

void loop() {
  for(int i = 0; i < nTasks; i++){
    if(TaskList[i].Count >= TaskList[i].MaxCount){
      TaskList[i].Count = 0;
      TaskList[i].CallbackFcn();
    }
    else{
      TaskList[i].Count++;
    }
  }
  delayMicroseconds(1000);
}
