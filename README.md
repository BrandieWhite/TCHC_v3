# TCHC_v3
This program operates the Thermal sensor Calibrator and Heat Capacity analyzer. It is two programs in one.
/*
  TMP Calibrator & Heat Capacity Analyzer
    Two mode Instrument:

Operation:   
1. Plug USB cable into laptop 
2. start Terminal program or Arduino IDE
3. Turn on TCHC unit  Power [green led will light] 
4. Select desired Function : Record or Playback. LED will confrim [Red/Green]
5. Select desired mode [mode 1=TMP CAL, mode 2 Heat CAp], LED will confrim mode [Yel/Blu]
6. If PlayBack is selected , enter "GO" in message box then press "Ctrl+Enter" and playback of Log file will begin.
7, If Record is selected , enter "GO" in message box then press "Ctrl+Enter" and user will be asked for both name and
   number of samples. Enter "name"in message box then press "Ctrl+Enter", then enter "number of samples"in message box then press "Ctrl+Enter",
   recording of Log file will begin.
8. Observe Blinking LED will confrim data is being logged, Terminal display will also show copy of LOG data
9. When the number of samples or playback is complete all files are closed and the system is then ready for next setup and ID.
10. When complete with session turm power switch off..

** Note:  A reset switch below "Data Log" led is provided if the system Hangs or is non-responcive.

  Functions: There are two main functions of the instrument.
             one is to record test data 
             the 2nd is to PlayBack recorderd data files.

  Mode 1:  TMP Calibrator
  Records 3 channels of sensor signals. One is a NIST standard Digital sensor[TMP117] the 2nd is a "Golden-Reference" [Gref]
  TMP Analog sensor and the thrid is the Analog sensor to be calibrated over the desired temprature range by determining its 
  gain and offset values when compaerd to the TMP117 & Gref. [Gref = "Golden Reference" TMP]
  The data foramt is :
  "ID"-->Timestamp[RTC], TMP117[degree C], Gref[volt], TMPtest[volt]
  
  The Sensor voltages are measured at pins 
                                    channel                Bread Board         Chassis
   TMP117   via I2C                                 BB-J-12,13,14,15       DB(15) -9,10,11,12
   Gref     AIN13/AIN12,             ch[6]               BB-J-8,9,10            DB(15) -1,2,3                           
   TMPtest AIN15/AIN14[TU]           ch[7]               BB-J-4,5,6             DB(15) -6,7,8

      //  Note default filename is "TMP_log.txt" 
  
  The circuit:
  - AD7124 control is connected on the MOSI, MISO, SCK and /SS pins (pin 10)
                                                                
                                                  channel         ADC         Bread Board      Chassis                                                              
                      ID data-----\              * User Input from Terminal
                      RTC---I2C----\
                      TMP117 --I2C-->------->  Record(SD)/Display..."Blink".....repeat
        Gref----------> AD7124 ----/                6          AIN13/AIN12,    BB-A-4,5,6       DB(15)_A -1,2,3
      TMP-test unit---> AD7124 ---/                 7          AIN15/AIN14,    BB-A-4,5,6       DB(15)_A -1,2,3
  
  ***********************************************************************************
  Mode 2: Heat Capacity of "test sample" relative to Distilled Water
  ***********************************************************************************
  Records 4 channels of analog signals. Two analog Temp. sensors  and two are precission resistors used as 
  precission local Heaters.
  Organized as two Independent Probes [A,B]
  The format is :
  ID-->Precision Time stamp, Prob_A [TMP sensor_a , VRheater_a] ,  Prob_B [TMP sensor_b , VRheater_b]  

   Where Probe_A is in Distilled Water,and Probe_B is in "Test sample" media.

  The Heat capacity of the Distilled water reference is assumed to be an  exact value as found in NIST official science references.
  The Test Sample Heat Capacity is then determined by a precission comparison to that value. The result is a
  high precission value "relative" to the Distilled reference at any given Temprature.

  A Poteniometer is used to adjust the current in the Heater resistor values and the ADC reads a precission voltage across it to 
  determin how much energy has been  added to the media.

  Note:
  All data collected in both modes are "RAW" and will need Post-Processing to determin the final values [Gain and Offset, Cp(test) ]
  
        //  Note default filename is "HCAP_log.txt" 

  The circuit:
  - AD7124 connected on the MOSI, MISO, SCK and /SS pins (pin 10)
                                                 channel         ADC         Bread Board      Chassis
      TMPa--------\  Probe_A                     ch[0]         AIN1/AIN0,    BB-A-4,5,6       DB(9)_A -1,2,3
      RESa --------\                             ch[1]         AIN3/AIN2,    BB-A-1,2         DB(9)_A -6,8
    RTC + ID-------->-----> Record(SD)/Display...Blink....repeat             
      TMPb---------/                             ch[2]         AIN5/AIN4     BB-A-13,14,15    DB(9)_B -1,2,3
      RESb -------/  Probe_B                     ch[3]         AIN7/AIN6     BB-A-9,10        DB(9)_B -6,8

 */

#include <NHB_AD7124.h>
#include "RTClib.h"
#include <SD.h>
#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>

RTC_DS3231 rtc;
Adafruit_TMP117  tmp117;


#define CH_COUNT 8 // 8 differential ADC channels   

const uint8_t csPin = 10;    // ADC chip select
const int chipSelect = 4;   // SD card strob
int funPin = 11; // Record = High,  Playbk = LOW
int modePin = 12; // Mode TMP Cal = HIGH,  HeatCapacity = LOW
int logPin = 6; // blink LED that new Log was written
// String  idname = "nc"; // Global var for ID number, reset value to user input
int nbrsam = 3; // # of data samples per experiment



Ad7124 adc(csPin, 4000000);
 
// The ADC filter select bits determine the filtering and ouput data rate
//     1 = Minimum filter, Maximum sample rate
//  2047 = Maximum filter, Minumum sample rate
//
// one can take readings at a SLOWER rate than
// the output data rate. (i.e. logging a reading every  30 seconds)
//
// NOTE: Actual output data rates in single conversion mode will be slower
// than calculated using the formula in the datasheet. This is because of
// the settling time plus the time it takes to enable or change the channel.
// 
int filterSelBits = 1024; //  max is 2047

void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

     if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
     }

    // Try to initialize TMP117
  if (!tmp117.begin()) {
    Serial.println("Failed to find TMP117 chip");
    while (1) { delay(10); }
  }
  Serial.println("TMP117 Found!");

  Serial.println ("AD7124 TMP sensor Calibration and Heat Capacity analysis");

  // Initializes the AD7124 device
  adc.begin();

  // Configuring ADC in Full Power Mode (Fastest) 
  adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true);

 
  // Set the "setup" configurations for different channels. There are 8 independent channel "setups"
  // in the ADC that can be configured. 
  // Each setup holds settings for: the reference used, the gain setting, filter type, and rate

  adc.setup[0].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // TMP-A            :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[1].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // RES-A            :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[2].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // TMP-B            :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[3].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // RES-B            :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[4].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // not used         :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[5].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // not used         :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[6].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // Gref             :      Internal reference, Gain = 1, Bipolar = True
  adc.setup[7].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);    // TMP DUT          :      Internal reference, Gain = 1, Bipolar = True

  // Filter settings for each setup. allows for seperate setup for each channel
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[1].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[2].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[3].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[4].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[5].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[6].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[7].setFilter(AD7124_Filter_SINC3, filterSelBits);

  // Set channels, i.e. what setup they use and what pins they are measuring on
  //  setchannel(ch#, setup#, "-"input, "+"input,true)
  adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true);   //Channel 0 - 
  adc.setChannel(1, 1, AD7124_Input_AIN2, AD7124_Input_AIN3, true);   //Channel 1 - 
  adc.setChannel(2, 2, AD7124_Input_AIN4, AD7124_Input_AIN5, true);   //Channel 2 - 
  adc.setChannel(3, 3, AD7124_Input_AIN6, AD7124_Input_AIN7, true);   //Channel 3 - 
  adc.setChannel(4, 4, AD7124_Input_AIN8, AD7124_Input_AIN9, true);   //Channel 4 - 
  adc.setChannel(5, 5, AD7124_Input_AIN10, AD7124_Input_AIN11, true); //Channel 5 -
  adc.setChannel(6, 6, AD7124_Input_AIN12, AD7124_Input_AIN13, true); //Channel 6 - 
  adc.setChannel(7, 7, AD7124_Input_AIN14, AD7124_Input_AIN15, true); //Channel 7 -
  
  // see if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  
 pinMode(funPin, INPUT_PULLUP);    // what functiion Rec/PlyBk  (Record= HIGH, PlyBk= LOW
 pinMode(modePin, INPUT_PULLUP );  // which mode    mode: HIGH =>TMPCAL  LOW=>HeatCap
 pinMode(logPin, OUTPUT);   // Blink at end of LOG cycle

 digitalWrite(6, HIGH);  // turn the LED off  

}

// -----------------RTC set.....ADC set.....SD card set....Serial I/O set ---------------------------------
// -----------------Start the DataLogging phase depending on the mode and function------------------------------

 void loop() {
  //*********************************************************************
  //***************Start Operation***************************************
  //*********************************************************************

  double readings[CH_COUNT];
 // make a clear set of strings for assembling new log ID, time  & data strings
   String timeString = ""; // clear string
   String idname = "";   // Input from user [alpha-numeric] diff for each sensor/analysis
   String idString = ""; // idname + --> as log file header
   String voltString = ""; // measurments from ADC , TMP117 seperate
   String logString = "";  // one entire LOG record entry per timestamp
   String gocmd = ""  ;  // Start Operations
 
  Serial.println("**********************************");
  Serial.println("* Select Function :  Rec/PlyBk   *");
  Serial.println("* Select Mode  : TMPcal/ HeatCap *");
  Serial.println("**********************************");
  Serial.println("**********************************");
  Serial.println("* When ready enter  ->           *");
  Serial.println("*   go + Ctrl + Enter            *");
  Serial.println("**********************************");

     //  On "GO" start
   while(Serial.available() == 0){
    }
   gocmd = Serial.readString();
   //Serial.println("gocmd= ");Serial.println(gocmd);
 


  //*********************************************************************
  //***************If PlyBk and mode= TCAL perform TMP_ file DUMP *****************************
  int funState = digitalRead(funPin);     // to determin Function
  int modeState = digitalRead(modePin);   // to determin mode
      Serial.print("funState= "); Serial.println(funState);
      Serial.print("modeState= "); Serial.println(modeState);


 // If PlayBack Function 
  if (funState == 0){ 
    if (modeState == 1){
          Serial.println("PLYBK/TMP ");  
      // Plybk TMP file
      File dataFile = SD.open("TMP_log.txt");
     // if the file is available, write to it:   Transfers TMP_log.txt
     if (dataFile) {
      while (dataFile.available()) {
      Serial.write(dataFile.read());
      }
     dataFile.close();
     }
     // if the file isn't open, pop up an error:
     else {
     Serial.println("error opening TMP_log.txt");
       }
       
    }
       
     
        //*************** *****************************
     if (modeState == 0){
              Serial.println("PLYBK/ HCAP ");  
         // PlyBk HCAP file
         File dataFile = SD.open("HCAP_log.txt");
         // if the file is available, write to it:
         if (dataFile) {
          while (dataFile.available()) {
          Serial.write(dataFile.read());
         }
         dataFile.close();
           }
         // if the file isn't open, pop up an error:
         else {
         Serial.println("error opening HCAP_log.txt");
              }
          
          }   
          
      
    }
  


    //*********************************************************************
    //***************RECORD Function****************************************
  
  if (funState == 1) { 

   Serial.println("**********************************");
   Serial.println("* Enter ID name into Message bar *");
   Serial.println("* Then press  'Ctrl+Enter'       *");
   Serial.println("**********************************");
   //  Get ID
   while(Serial.available() == 0){
    }
   idname = Serial.readString();
      //   Serial.print("name= "); Serial.println(idname);

   Serial.println("**********************************");
   Serial.println("* Enter # of samples  1 to 36000 *");
   Serial.println("* Then press  'Ctrl+Enter'       *");
   Serial.println("* Operation will then begin      *");
   Serial.println("**********************************");
   //  Get nbrsam
   while(Serial.available() == 0){
    }
   String nbrsam = Serial.readString();
    //     Serial.print("# of samples= "); Serial.println(nbrsam);
    int nbr = nbrsam.toInt();
  Serial.print("nbr= "); Serial.println(nbr);
  
  // make the following number of LOG entries
      for(int i = 1; i <= nbr; i++) {
    idString  = "";
   
   // Build a new TimeStamp string for each entry
    timeString = ""; // clear old value
   DateTime now = rtc.now();
    timeString += String("T= ");
    timeString += String(now.year(), DEC);
    timeString += String(':');
    timeString += String(now.month(), DEC);
    timeString += String(':');
    timeString += String(now.day(), DEC);
    timeString += String(':');
    timeString += String(now.hour(), DEC);
    timeString += String(':');
    timeString += String(now.minute(), DEC);
    timeString += String(':');
    timeString += String(now.second(), DEC);
    timeString += String(':');
    delay(12);
    // Timestamp string is done
     //     Serial.print("timestamp= "); Serial.println(timeString);


   
   // Read all ADC ports, display and LOG depends on function and mode 
   readings[0] = adc.readVolts(0);
   readings[1] = adc.readVolts(1); 
   readings[2] = adc.readVolts(2);
   readings[3] = adc.readVolts(3);
   readings[4] = adc.readVolts(4);
   readings[5] = adc.readVolts(5);
   readings[6] = adc.readVolts(6);
   readings[7] = adc.readVolts(7);
     //    Serial.println("ADC is read ");  

   //  Get TMP117 data  for mode 1 
    sensors_event_t temp; // create an empty event to be filled
   tmp117.getEvent(&temp); //fill the empty event object with the current measurements

   // Serial.print("T TMP117 =  "); Serial.print(temp.temperature, 3);Serial.print(" degrees C");
   //  Serial.print("  ");
   delay(10);
     //    Serial.println("read tmp117 ");


   // **********************************************************************
   // One now has the  ID#, Time stamp, TMPCAL readings and Probe-A & -B reading [even if not used in selected mode]
   // mode-1 log => ID--> Timestamp,  TMP117, Gref, TMPtest     ---------------> TMP_log.txt
   // mode-2 log => ID--> Timestamp,  TMP-A, RES-A, TMP-B, RES-B --------------> HCAP_log.txt

   // **********************************************************************
   //  mode-1 = Record function and TMPCAL 
   //  Serial.print("Record mode TMP ");


   if (modeState == 1) { 

     idString  +=  String("TMP__") + idname + String(" --> ");

     logString = ""  ; // clear
     // Merge strings   ID + Time + data  TMP version
    logString +=  idString + timeString ;
   
  	logString += String(": TMP117 =") + String(temp.temperature, 3) ;
  	logString += String(": Gref= ") + String(readings[6] ,6 ) ;
  	logString += String(": TMP = ") + String(readings[7] ,6 ) ;
    logString += String(":: ");
  
   // open the mode 1 file. Records only TMP Cal data here
   File data1File = SD.open("TMP_log.txt", FILE_WRITE);

   // if the file is available, write to it:
   if (data1File) {

    data1File.println(logString);  // log data to file
    data1File.close();

    //*************** print to the serial port **************************
    Serial.println(logString);   // Send log to Terminal/Display on screen
   }
    // if the file isn't  open, pop up an error:
   else {
    Serial.println("error opening TMP_log.txt");
   }
   // ****************Blink ******************
   digitalWrite(logPin, LOW);   // turn the LOG LED on  
   delay(200);                  // wait  
   digitalWrite(logPin, HIGH);  // turn the LED off  
   delay(10); 
   // end of TMPCAL
    }    

   
  


    // **********************************************************************
    // If Mode 2  LOG and display
    if (modeState == 0) { 
      // Serial.println("REC + HCAP "); Serial.println(funState); Serial.println(modeState); 
       idString  += String("HCAP_") + idname + String(" --> ");   // ID string



      logString = "";   // Clear string
    // Merge LOG record = Time + ID + data strings  HCAP version
      logString += idString + timeString ;
      
      logString += String(": TMP-A = ");   logString += String(readings[0],  6);  //Temp sensor A
      logString += String("  RES-A = ");  logString += String(readings[1],  6);  //Heater Resistor A
      logString += String(": TMP-B = "); logString += String(readings[2],  6);  //Temp sensor B
      logString += String(" RES-B = ");  logString += String(readings[3],  6);  //Heater Resistor B
      logString += String(" :: ");                                                                                                                              

     

      // open the file. Records Heat Capacity data only
      File data2File = SD.open("HCAP_log.txt", FILE_WRITE);

     // if the file is available, write to it:
      if (data2File) {
      data2File.println(logString);    // LOG to file
      data2File.close();
	
      // ************************print to the serial port*********************
      Serial.println(logString);      // Display copy to screen
     }
      // if the file isn't open, pop up an error:
      else {
      Serial.println("error opening HCAP_log.txt");
      }
      // ****************Blink ******************
      digitalWrite(logPin, LOW);   // turn the LOG LED on  
      delay(200);                  // wait  
      digitalWrite(logPin, HIGH);  // turn the LED off  
      delay(10);   
	    // end of HCAP
      }  
    }
 }
 }
