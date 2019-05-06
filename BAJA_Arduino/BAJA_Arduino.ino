#include <RadioHead.h>

#include <RH_RF95.h>


/*
 * Contributed Libraries
 * Use "Manage Libraries" menu option to download these
 */
#include <FreqMeasure.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdlib.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <SPI.h>

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

/* Arduino Micro */
#if defined (__AVR_ATmega32U4__) 
  /* G0 Pin - Has to be an external interrupt pin (RFM95_INT)*/
  #define RFM95_INT     0
  #define RFM95_CS      4 
  #define RFM95_RST     3
  #define LED           13
#endif

/* Arduino Uno | Arduino Nano */
#if defined (__AVR_ATmega328P__) 
  /* G0 Pin - Has to be an external interrupt pin (RFM95_INT)*/
  #define RFM95_INT     3 
  #define RFM95_CS      4  
  #define RFM95_RST     2  
  #define LED           13
#endif

/*
 * Debugging defines
 * 
 * Commenting out the define will allow debugging messages over serial
 */
 #define DEBUGGING_SETUP
 #define DEBUGGING_DAC
 #define DEBUGGING_RPM

 #include <Wire.h>

 /* User defined parameters for DAC */
 #define  DAC_I2C_ADDRESS         0x62
 #define  DAC_MIN                 0.00
 #define  DAC_MAX                 5.00

 /* User defined parameters for RPM  */
 #define  RPM_INPUT_PIN           13
 #define  RPM_SAMPLING_AVERAGE    2
 #define  RPM_MIN                 10
 #define  RPM_MAX                 50

 /* RFM95 Frquency 915.0, 434.0, or 868.0 MHz */
 #define RF95_FREQ                915.0

 RH_RF95 rf95(RFM95_CS, RFM95_INT);
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
    Serial.print(F("Initializing FreqMeasure on pin "));
    Serial.println(RPM_INPUT_PIN);
   #endif
   FreqMeasure.begin();
   pinMode(13, INPUT_PULLUP); //THIS HAS TO BE A NUMBER. NO VARIABLES/DEFINES. MESSES THIS UP FOR SOME REASON

   /*
    * Configure Wire (I2C) for the DAC
    */
   #ifdef DEBUGGING_SETUP
    Serial.println(F("Initializing Wire for the DAC"));
   #endif
   DAC.begin(DAC_I2C_ADDRESS);

   //Default the DAC voltage to 0
   DAC.setVoltage(0, false);

   //Initialize the IMU
   if(!IMU.begin()){
    Serial.println(F("Failed to connect to the IMU!"));
   }

   delay(1000);
   IMU.setExtCrystalUse(true);
   
   /* 
    * Configure RFM9X LoRa radio 
    */
   #ifdef DEBUGGING_SETUP
    Serial.println(F("Initializing LoRa TX Test!"));
   #endif 

   pinMode(RFM95_RST, OUTPUT);
   digitalWrite(RFM95_RST, HIGH);
   delay(10);

   /* Manual Reset */
   digitalWrite(RFM95_RST, LOW);
   delay(10);
   digitalWrite(RFM95_RST, HIGH);
   delay(10);

   while (!rf95.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }
  
   Serial.println(F("LoRa radio init OK!"));

  /* Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM */
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  }

  Serial.print(F("Set Freq to: ")); Serial.println(RF95_FREQ);

  /* 
   * The default transmitter power is 13dBm, using PA_BOOST.
   * If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
   * you can set transmitter powers from 5 to 23 dBm:
   */
  rf95.setTxPower(23, false);

   #ifdef DEBUGGING_SETUP
    Serial.println(F("Initialization done"));
   #endif
}

float CurrentFrequency = 0;
float DACVoltage = 0;

/* Converts frequency to a DAC voltage value */
float DAC_FrequencyToVoltage(float Frequency){
  float RPM_Range = RPM_MAX - RPM_MIN;
  float DAC_Range = DAC_MAX - DAC_MIN;
  float RPM_Normalized = (Frequency - RPM_MIN) / RPM_Range;
  float Voltage = (RPM_Normalized * DAC_Range) + DAC_MIN;
  return Voltage;
}

/* Converts voltage value to DAC voltage output  */
uint16_t DAC_VoltageToValue(float Voltage){
  float ratio = 4095/5.0;
  float Value = Voltage * ratio;
  return (uint16_t)(Value);
}

/* Update DAC voltage output */
void UpdateDAC(){
  DACVoltage = DAC_FrequencyToVoltage(CurrentFrequency);
  DAC.setVoltage(DAC_VoltageToValue(DACVoltage), false);
}

/* Updats frequency of hall effect sensor */
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
    Serial.print(F("Frequency set to "));
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

int index = 0;
char ByteArray[RH_RF95_MAX_MESSAGE_LEN] = {0};
char Tab[] = " ";

// Adds data to array
void AddByteArray(char BA[], int size){
  strcat(ByteArray, BA);
  strcat(ByteArray, Tab);
  index = size;
}

/* Reads DOF sensor and prints data out to serial monitor */
void ReadDOFAndPrint(){
  //delay(1000);
  
  sensors_event_t event;
  IMU.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

  char radioPacket[10];
  String result = "";

  memset(ByteArray, 0, sizeof(ByteArray));

  Serial.print(F("\tX: "));
  Serial.print(event.orientation.x, 4);
  Serial.print(F("\tY: "));
  Serial.print(event.orientation.y, 4);
  Serial.print(F("\tZ: "));
  Serial.print(event.orientation.z, 4);

  sensors_event_t AccEvent;
  IMU.getEvent(&AccEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(F("\taX: "));
  Serial.print(AccEvent.acceleration.x, 4);
  Serial.print(F("\taY: "));
  Serial.print(AccEvent.acceleration.y, 4);
  Serial.print(F("\taZ: "));
  Serial.println(AccEvent.acceleration.z, 4);

  // Converts float to string 
  dtostrf(CurrentFrequency, 1, 1, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(event.orientation.x, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(event.orientation.y, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(event.orientation.z, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(AccEvent.acceleration.x, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(AccEvent.acceleration.y, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  dtostrf(AccEvent.acceleration.z, 3, 4, radioPacket);
  AddByteArray(radioPacket, sizeof(radioPacket));
  
  Serial.print(F("In the packet: ")); Serial.println(ByteArray);

  Serial.println(F("Sending..."));
  rf95.send((uint8_t *)ByteArray, sizeof(ByteArray));
  
  Serial.println(F("Waiting for packet to complete...")); 
  rf95.waitPacketSent();
  
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println(F("Waiting for reply..."));
  if (rf95.waitAvailableTimeout(100))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }

}

typedef struct sTask_tag{
  uint32_t Count;
  uint32_t MaxCount; //Period of task in milliseconds
  void (*CallbackFcn)(void);
} sTask;

/* Task Scheduler - If new task, add task to scheduler */
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
  },
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

