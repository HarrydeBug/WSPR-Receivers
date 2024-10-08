/*
/Firmware for ZachTek Multi Band WSPR Receiver, product number 1055
/This firmware makes the receiver emulate an ICOM IC-741 Radio so it can be CAT controlled from a software like Fldigi, WSJT-X etc
/
/
/To compile and upload this firmware follow the instructions below:
/In the Arduino IDE first install the correct Board type:
/Start Arduino and open File, Preferences window.
/Enter https://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
/Open Boards Manager from Tools > Board menu and search for esp8266, Install the latest version.
/
/Pick the correct board type,
/Menu  Tools, Board, esp8266, Generic ESP82855 Module.
/Tools, Upload Speed, 921600
/Tools, Crystal Frequency, 26MHz
/Tools, Reset Method, dtr (aka nodemcu)
/
/Pick the correct Serial port
/
/Open the lid of the Receiver and make sure the switch between the USB and Audio connector is set to "prog." position
/Compile and upload
/
/Put the switch back to the "Normal (CAT)" position before you operate it with a software that will send CAT commands.
/If you forget the switch in the Prog mode the Status LED will blink ON/OFF once a second to indicate you are in programming and calibration mode
/In this mode the unit will only accept ZachTek serial API commands that can be used to set up inital parameters like the
/Name of the unit, the calibrated freuency of the reference oscillator etc. See the Serial API reference document for a full list of commands.
/This data will be be saved in EEPROM by the command - 
/The EEPROM is read at startup and used by the receiver when it is in CAT mode. 
/
*/


/*
/TO-DO
/ Skicka {DFR}, {DDE} och {DGF} vid VFO byte
/Bryt ut rutinen som skickar DFR ocg DGF till subrutin så den kan kallas på från flertalet positioner ikoden. 
*/

const uint8_t  SoftwareVersion =  1; //0 to 255. 0=Beta
const uint8_t  SoftwareRevision = 0; //0 to 255


// Product model. #1055 Multi-Band WSPR Receiver         =1055
// Product model. #1057 HF SSB Receiver                  =1057

const uint16_t Product_Model                           = 1055;

#include "IC746.h"
int ln = 0;
#include "Wire.h"      //Arduino I2C library
#include <EEPROM.h>   // Arduino EEPROM lib.
#include <SoftwareSerial.h>
SoftwareSerial Debug(14,12); // RX, TX

IC746 radio = IC746();


//Constants

const uint64_t Min_Freq=100000;    //Lower Frequency limit of the Receiver 100kHz  (In Hertz)
const uint64_t Max_Freq=28800000;  //Upper Frequency Limit of the Receiver 28.8MHz  (In Hertz)

//Si5351 register data
#define SI_R_DIV_1 0b00000000 // R-division ratio definitions
#define SI_R_DIV_128 0b01110000
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_MS_0 42
#define SI_PLL_RESET 177
#define SI_CLK0_CONTROL 16 
#define SI_CLK_SRC_PLL_A 0b00000000


// variables
//unsigned long freq =  14095600l;;
//unsigned long bfreq = 14125000l;

boolean splitActive = false;
uint8_t Si5351I2CAddress;            //The I2C address on the Si5351 as detected on startup
#define StatusLED    0               //Yellow LED that indicates current status. 
boolean LEDValue;                    //Keep tracks of the HIGH/LOW state of the StatusLED pin
#define PROG_Switch   2              //IO Pin 2 used to test what postion of the CAT/Programing switchs. 
#define SidebandSelction_IOPin   13  //I/O pin that controls if the receiver is decoding Upper or Lower Sideband, LOW=Lower sideband HIGH=Upper Sideband

unsigned long CATReceivedTime=0; //At what time did we recveive a CAT command, used to blink the Status LEd for some time whenever we receive a CAT command 
//#define VFO_A 0
//#define VFO_B 1


const uint8_t SerCMDLength = 50; //Max number of char on a command in the SerialAPI

// radio modes
#define MODE_LSB 00
#define MODE_USB 01
byte mode = MODE_USB;

#define PTT_RX 0
#define PTT_TX 1
byte ptt = PTT_RX;

enum E_Detector             //Receiver detector mode, Upper or lower sideband
{ 
  USB, 
  LSB 
};

enum E_VFOs                 //Receiver VFOs
{ 
  VFO_A, 
  VFO_B 
};



// Data structures
struct S_UserData
{
  char        Name[40];          //Optional Name of the device.
  E_VFOs      ActiveVFO;         //The active VFO - A or B
  uint64_t    VFO_A_Freq;        //VFO A Frequency in Hertz
  E_Detector  VFO_A_Detector;    //VFO A Recive detector - Upper or Lower Sideband
  uint64_t    VFO_B_Freq;        //VFO B Freq.
  E_Detector  VFO_B_Detector;    //VFO B Detector
  uint32_t    ExtRefFreq;        //The frequency of the external reference oscillator in Hz, usually 10000000 (10MHz), Not available on all models
};


struct S_FactoryData
{
  uint8_t  HW_Version;           // Hardware version
  uint8_t  HW_Revision;          // Hardware revision
  uint32_t RefFreq;              //The exact frequency in Hertz of the built in reference TCXO (Temperature Controlled Crystal Oscillator) as determined by calibration and saved in EEPROM
};


#define FactorySpace true             //FactorySPace=EEPROM Space reserved for Factory data that can not be changed by the user
#define UserSpace    false

S_UserData    UserData;              //A data structure that holds all relevant data for the reciver like LO frequency etc
S_FactoryData FactoryData;           //A data structure that holds all the factory data like reference frequency, hardware versionetc, this can not be changed by the user  

void setup() {


   EEPROM.begin(512);
   Wire.begin();

  // preload the vars in the cat library
  radio.addCATPtt(catSetPtt);
  radio.addCATGetPtt(catGetPtt);
  radio.addCATAtoB(catVfoAtoB);
  radio.addCATSwapVfo(catSwapVfo);
  radio.addCATsplit(catSetSplit);
  radio.addCATFSet(catSetFreq);
  radio.addCATMSet(catSetMode);
  radio.addCATVSet(catSetVFO);
  radio.addCATGetFreq(catGetFreq);
  radio.addCATGetMode(catGetMode);
  radio.addCATSMeter(catGetSMeter);

  // now we activate the library
  radio.begin(9600,SERIAL_8N1);
  Debug.begin(9600);
  
  Debug.println(F("Restart"));
  Debug.print(F("Firmware version "));
  Debug.print(SoftwareVersion);
  Debug.print((":"));
  Debug.println(SoftwareRevision);
  
  // Use the Yellow LED as Status indicator
  pinMode(StatusLED, OUTPUT); // Use the Yellow LED as Status indicator but also as an output to test the postion of the CAT/Programing switch
  pinMode (PROG_Switch, INPUT);//Input to test the postion of the CAT/Programing switch
  pinMode(SidebandSelction_IOPin, OUTPUT);  //Controls what Sideband to receive
  
  LEDBlink(3);  //Blink the status LED a few time to indicate a reboot, we cant send to many as it would delay start up and case problems with any CAT software that are expecxting a timely respons  

  if (!SwitchIsInCATMode ()) //the switch is not in CAT mode so we can send a few ZachTek Serial API info strings 
  {
  LEDBlink(7);    //Send a few more startup blinks
  Serial.println("");
  Serial.println(F("{MIN} Startup"));
  Serial.print(F("{MIN} Firmware version "));
  Serial.print(SoftwareVersion);
  Serial.print((":"));
  Serial.println(SoftwareRevision);
  }
  
  if (LoadFromEPROM(FactorySpace)==false) //Read Factory data from EEPROM
  { //if Configuration and Calibration has not been run set some default values.
     if (Product_Model == 1055)    //If WSPR RX model
    {
      FactoryData.HW_Version=1;    // Hardware Major Version
      FactoryData.HW_Revision=7;   // Hardware Revision
    }
    FactoryData.RefFreq=26000000UL;  //Set Reference osillator Frequency to 26MHz, this needs to be calibrated, it's just so we have something before the calibration have been run and stored here. 

    Debug.println (F("No Calibration data found!!  Run Factory calibration"));
    Debug.println (F(""));
     if (!SwitchIsInCATMode ()) //the switch is not in CAT mode so we can send a few ZachTek Serial API info strings 
    {
      Serial.println(F("{MIN} No factory data found !"));
      Serial.println(F("{MIN} You need to run factory setup to complete the configuration, guessing on calibration values for the internal frequency reference"));
      Serial.println(F("{MIN} Your frequency accuracy will be poor until you calibrate"));
    }  
  }  
    
  if (LoadFromEPROM(UserSpace)==false) //Read User data from EEPROM
  { //if Configuration and Calibration has not been run set some default values. 
     if (Product_Model == 1055)    //If WSPR RX model
    {
     UserData.Name[0] = 'W'; UserData.Name[1] = 'S'; UserData.Name[2] = 'P'; UserData.Name[3] = 'R';
     UserData.Name[4] = ' '; UserData.Name[5] = 'R'; UserData.Name[6] = 'X'; UserData.Name[7] = 0;
    }
    UserData.ActiveVFO=VFO_A;         //VFO A is active 
    UserData.VFO_A_Freq=1409560000UL;  //Set VFO A to 7.1MHz  (In centi Hertz)
    UserData.VFO_A_Detector=USB;      //Set VFO A to Upper Sideband reception
    UserData.VFO_B_Freq=1810460000UL; //Set VFO B to 14.2MHz (in centi Hertz)
    UserData.VFO_B_Detector=USB;      //Set to Upper Sideband reception
    UserData.ExtRefFreq=10000000;     //Set Ext  ref to 10MHz (In Hertz), not all products have an external reference input but the share the same firmware

    Debug.println (F("No User data found"));
    if (!SwitchIsInCATMode ()) //The switch is not in ICOM CAT mode - (It's in Programing and Setup postion) so we can send a few ZachTek Serial API info strings 
     {
       Serial.println(F("{MIN} No user data was found, setting default values"));
     }  
  }
  DetectSi5351I2CAddress();
  SetLO();  
  SetDetector();
}

void loop() {
boolean SwitchTest ;
static unsigned long MillisOld = 0;
static unsigned long MillisNow = 0;
   
  if (SwitchIsInCATMode ()) //the switch is in CAT mode
  {
   radio.check(); // Listen to CAT commands 
   MillisNow=millis();
   if ( (MillisNow - CATReceivedTime) < 100UL ) //If a CAT command has been received light up the LED for 100mS  
   {
     SetStatusLED(LOW);//LOW = LED ON
   }
   else
   {
     SetStatusLED(HIGH);//LOW = LED OFF
     delay (5);  //No New CAT command, pause for a while
   }
  }
  else                       //Switch is in programing mode - listen for ZachTek APIs instead of ICOM CAT commands
  { 
    if (Serial.available()) 
    {  //Handle  Serial API request from the PC
      Za_DoSerialHandling();
    } 
    MillisNow=millis();
    if (((MillisNow - MillisOld) > 500UL) || (MillisNow < MillisOld))//If 0.5 Seconds have gone, toggle LED to make it blink in Programing mode
       //millis() funtion can wrap around after 50 days so handle that by checking if millis is smaller than the old stored time  
    {
     ToggleStatusLED(); 
     MillisOld=millis();
    }     
   delay(5);
  }
}




/* ------------------------------------------------------------------------------------------
/ CAT Decoding 
/ -------------------------------------------------------------------------------------------
*/

// function to run when we must put radio on TX/RX
void catSetPtt(boolean catPTT) {
  // the var ptt follows the value passed, but you can do a few more thing here
  if (catPTT) {
    ptt = PTT_TX;
  } else {
    ptt = PTT_RX;
  }
CATReceivedTime=millis();
}

boolean catGetPtt() {
  if (ptt == PTT_TX) {
    return true;
  } else {
    return false;
  }
CATReceivedTime=millis();  
}

// function to run to toggle Split mode on and off
void catSetSplit(boolean catSplit) {
  // the var ptt follows the value passed, but you can do a few more thing here
  if (catSplit) {
    splitActive = true;
  } else {
    splitActive = false;
  }
 CATReceivedTime=millis();
}

// function to run when
void catSwapVfo() {
  // Swap the active VFO
  if (UserData.ActiveVFO == VFO_A) 
  {
    UserData.ActiveVFO=VFO_B;
  } 
  else 
  {
    UserData.ActiveVFO=VFO_A;
  }
  SetLO();
  SetDetector ();
  CATReceivedTime=millis();
}

// function to set a freq from CAT
void catSetFreq(long f) 
{
  if (f > Max_Freq) f = Max_Freq;
  if (f < Min_Freq) f = Min_Freq;
  if (UserData.ActiveVFO == VFO_A) 
  { //VFO A is active 
    UserData.VFO_A_Freq=f *100ULL;
  } 
  else 
  { //VFO B is active 
    UserData.VFO_B_Freq=f *100ULL;
  }
  SetLO();
  CATReceivedTime=millis();
}

// function to set the mode from the cat command
void catSetMode(byte m) {
  if (m == CAT_MODE_LSB)  //Lower Sideband 
  {
    if(UserData.ActiveVFO==VFO_A)
    {//VFO A is active 
    UserData.VFO_A_Detector=LSB; 
    }
    else
    {
     UserData.VFO_B_Detector=LSB; 
    } 
  } 
  else //Upper sideband
  {
    if(UserData.ActiveVFO==VFO_A)
    {//VFO A is active 
    UserData.VFO_A_Detector=USB; 
    }
    else
    {
     UserData.VFO_B_Detector=USB; 
    } 
  }
  SetDetector ();
  CATReceivedTime=millis();
}


// function to set the active VFO from the cat command
void catSetVFO(byte v) {
  if (v == CAT_VFO_A) {
    UserData.ActiveVFO = VFO_A;
  } else {
    UserData.ActiveVFO = VFO_A;
  }
  SetLO();
  SetDetector ();
  CATReceivedTime=millis();
}

// Function to make VFOS the same
void catVfoAtoB() {
  if (UserData.ActiveVFO == VFO_A) {
    UserData.VFO_B_Freq =UserData.VFO_A_Freq;
    UserData.VFO_B_Detector=UserData.VFO_A_Detector;
  } else {
    UserData.VFO_A_Freq =UserData.VFO_B_Freq;
    UserData.VFO_A_Detector=UserData.VFO_B_Detector;
  }
 CATReceivedTime=millis();
}

// function to pass the freq to the cat library
long catGetFreq() 
{
  // this must return the freq as an unsigned long in Hz, you must prepare it before
  long f;

  if (UserData.ActiveVFO == VFO_A) {
    f = UserData.VFO_A_Freq / 100ULL;  //Stored as CentiHertz but we must pass it as Hertz
  } else {
    f = UserData.VFO_B_Freq / 100ULL;
  }
  return f;
  CATReceivedTime=millis();
}

// function to pass the mode to the cat library
byte catGetMode() 
{
  // this must return the mode in the what the CAT protocol expect it
  byte catMode;
  E_Detector Detector;
  
  if(UserData.ActiveVFO==VFO_A)
    {//VFO A is active 
    Detector=UserData.VFO_A_Detector;  
    }
    else
    {
     Detector=UserData.VFO_B_Detector;  
  } 
  
  switch (Detector)
  {
    case USB:
      catMode = CAT_MODE_USB;
      break;

    case LSB:
      catMode = CAT_MODE_LSB;
      break;
  } 
  return catMode;
 CATReceivedTime=millis();
}



// function to pass the smeter reading in RX mode, fake it, not a real S-meter reading.
byte catGetSMeter() 
{
  static int s = 0;
  static byte counter = 0;

  if (counter == 5) {
    counter = 0;
    if (s == 15) {
      s=0;
    } else {
      s++;
    }
  } else {
    counter++;
  }
  //  Return 0-9, 10=S9+10 - 15=S9+60

  return byte(s);
 CATReceivedTime=millis();
}




/* -----------------------------------------------------------------------------------------------------
/ ZachTek Serial API handling
/ ------------------------------------------------------------------------------------------------------
*/





//Parts from NickGammon Serial Input example
//http://www.gammon.com.au/serial
void Za_DoSerialHandling()
{
  static char SerialLine[SerCMDLength]; //A single line of incoming serial command and data
  static uint8_t input_pos = 0;
  char InChar;
  //PCConnected = true;
  while (Serial.available () > 0)
  {
    InChar = Serial.read ();
    switch (InChar)
    {
      case '\n':   // end of text
        SerialLine [input_pos] = 0;  // terminating null byte
        // terminator reached, process Command
        Za_DecodeSerialCMD (SerialLine);
        // reset buffer for next time
        input_pos = 0;
        break;

      case '\r':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (input_pos < (SerCMDLength - 1))
          SerialLine [input_pos++] = InChar;
        break;

    }  // end of switch
  } // end of processIncomingByte
}



//Serial API commands and data decoding
void Za_DecodeSerialCMD(const char * InputCMD) 
{
 
  char CharInt[13];
  bool EnabDisab;
  uint32_t i;
  E_Detector Detector;
  E_VFOs VFO;

  if ((InputCMD[0] == '[') && (InputCMD[4] == ']')) { //Valid ZachTek API command

    if (InputCMD[1] == 'C') {  //Commmand

      //Current Mode
      if ((InputCMD[2] == 'C') && (InputCMD[3] == 'M')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8] == 'S') {
            SetLO();
          }
        }//Set Current Mode
        else //Get
        {
           Serial.println (F("{CCM} S"));
        }//Get Current Mode
      }//[CCM]

      
      //Check what frequency Reference is in use, External or Internal  [CCR]
      if ((InputCMD[2] == 'C') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'G')  //Get option
          {
          Serial.print (F("{CCR} "));  
          //This hardware only has an internal frequency reference
          Serial.println("I");
        }//Get 
      }//[CCR]

      
      //Store Current configuration data to EEPROM [CSE]
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          SaveToEEPROM(UserSpace);
          Serial.println(F("{MIN} Configuration saved"));
        }
      }//[CSE]
      exit;
    }//End of command section

    //Data
    if (InputCMD[1] == 'D') {
       
      //Current VFO in use
      if ((InputCMD[2] == 'V') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8]=='A')
            {
            VFO=VFO_A;
            Debug.println("VFO A");
            }
          else
            {
             VFO=VFO_B;
             Debug.println("VFO B");
            }   
          UserData.ActiveVFO=VFO;  
          SetLO();         
          SetDetector();    
          Za_Answer_DFR();   
        }
        else //Get
        {
          Serial.print (F("{DVF} "));
          if (UserData.ActiveVFO == VFO_A) 
            {
            Serial.println ("A"); 
            }
          else
            {
            Serial.println ("B");  
          }  
        }
      }//Current VFO
  


      //Name
      if ((InputCMD[2] == 'N') && (InputCMD[3] == 'M')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 38; i++) {
            UserData.Name[i] = InputCMD[i + 8];
          }
            UserData.Name[39] = 0;
        }
        else //Get
        {
          Serial.print (F("{DNM} "));
          Serial.println (UserData.Name);
        }
      }//Name

      //Current Detector
      if ((InputCMD[2] == 'D') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          if (InputCMD[8]=='L' || InputCMD[9]=='S' || InputCMD[10]=='B' )
            {
            Detector=LSB;
            Debug.println("LSB");
            }
          else
            {
             Detector=USB; 
             Debug.println("USB");
            }   
          if (UserData.ActiveVFO == VFO_A) 
            { //VFO A is active
            UserData.VFO_A_Detector=Detector;
            }
          else
            {  //VFO B is active
            UserData.VFO_B_Detector=Detector;
          }
          SetDetector();    
        }
        else //Get
        {
          Serial.print (F("{DDE} "));
          if (UserData.ActiveVFO == VFO_A) 
            { //VFO A is active
            Detector=UserData.VFO_A_Detector;
            }
          else
            {  //VFO B is active
            Detector=UserData.VFO_B_Detector;
          }
          if (Detector==LSB)
            {
            Serial.println ("LSB");
          }
          else
            {
            Serial.println ("USB");
          }
        }
      }//Current Detector

      //Generator Frequency. This command is here so that one can use the configuration software for the WSPR TRANSMITTER and use the signal generator funtion to set the receive frequency
      if ((InputCMD[2] == 'G') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 11; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[12] = 0;
          if (UserData.ActiveVFO == VFO_A) 
            { //VFO A is active 
            UserData.VFO_A_Freq=StrTouint64_t(CharInt);
            } 
          else 
            { //VFO B is active 
            UserData.VFO_B_Freq=StrTouint64_t(CharInt);
            }
          SetLO();
        }
        else //Get
        {
          Za_Answer_DFR();          
        }
      }//Generator Frequency

      //Receive Frequency. This is the Tuning command that sets the receive frequency
      if ((InputCMD[2] == 'F') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 11; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[12] = 0;
          if (UserData.ActiveVFO == VFO_A) 
            { //VFO A is active 
            UserData.VFO_A_Freq=StrTouint64_t(CharInt);
            } 
          else 
            { //VFO B is active 
            UserData.VFO_B_Freq=StrTouint64_t(CharInt);
            }
          SetLO();
        }
        else //Get
        {
          Za_Answer_DFR();          
        }
      }//Receive Frequency

      //External Reference Oscillator Frequency
      if ((InputCMD[2] == 'E') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 8; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[9] = 0;
          UserData.ExtRefFreq  = StrTouint64_t(CharInt);
        }
        else //Get
        {
          Serial.print (F("{DER} "));
          Serial.println (uint64ToStr(UserData.ExtRefFreq, true));
        }
      }//External Reference Oscillator Frequency

      exit;
    }//Data

    //Factory data
    if (InputCMD[1] == 'F') {

      //Product model Number
      if ((InputCMD[2] == 'P') && (InputCMD[3] == 'N')) {
        if (InputCMD[6] == 'G')
        { //Get option
          Serial.print (F("{FPN} "));
          if (Product_Model < 10000) SerialPrintZero();
          Serial.println (Product_Model);
        }
      }//Product model Number

      //Hardware Version
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3]  = 0;
          FactoryData.HW_Version = atoi(CharInt);
        }//Set
        else //Get Option
        {
          Serial.print (F("{FHV} "));
          if (FactoryData.HW_Version < 100) SerialPrintZero();
          if (FactoryData.HW_Version < 10) SerialPrintZero();
          Serial.println (FactoryData.HW_Version);

        }
      }//Hardware Version

      //Hardware Revision
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'S') { //Set option
          CharInt[0] = InputCMD[8]; CharInt[1] = InputCMD[9]; CharInt[2] = InputCMD[10];
          CharInt[3]  = 0;
          FactoryData.HW_Revision = atoi(CharInt);
          //Serial.println (' ');
        }//Set
        else //Get Option
        {
          Serial.print (F("{FHR} "));
          if (FactoryData.HW_Revision < 100) SerialPrintZero();
          if (FactoryData.HW_Revision < 10) SerialPrintZero();
          Serial.println (FactoryData.HW_Revision);
        }
      }//Hardware Revision

      //Software Version
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSV} "));
          if (SoftwareVersion < 100) SerialPrintZero();
          if (SoftwareVersion < 10) SerialPrintZero();
          Serial.println (SoftwareVersion);
        }
      }//Software Version

      //Software Revision
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSR} "));
          if (SoftwareRevision < 100) SerialPrintZero();
          if (SoftwareRevision < 10) SerialPrintZero();
          Serial.println (SoftwareRevision);
        }
      }//Software Revision

      //Reference Oscillator Frequency
      if ((InputCMD[2] == 'R') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 8; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[9] = 0;
          FactoryData.RefFreq = StrTouint64_t(CharInt);
          Debug.print("Factory Reference Frequency =");
          Debug.println(CharInt);
          SetLO();
        }
        else //Get
        {
          Serial.print (F("{FRF} "));
          Serial.println (uint64ToStr(FactoryData.RefFreq, true));
        }
      }//Reference Oscillator Frequency

      //Store Current Factory configuration data to EEPROM
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          SaveToEEPROM(FactorySpace);
          Serial.println(F("{MIN} Factory data saved"));
        }
      }

      exit;
    }//Factory
  } 
}

void Za_Answer_DFR()
{
  Serial.print (F("{DFR} "));
  if (UserData.ActiveVFO == VFO_A) 
    { //VFO A is active 
    Serial.println (uint64ToStr(UserData.VFO_A_Freq, true)); 
    Serial.print (F("{DGF} "));  // Also send DFR Status to make it compatible with the TRANSMITTER Config software
    Serial.println (uint64ToStr(UserData.VFO_A_Freq, true));
    }
  else
    {
    Serial.println (uint64ToStr(UserData.VFO_B_Freq, true)); 
       Serial.print (F("{DGF} "));  // Also send DFR Status to make it compatible with the TRANSMITTER Config software
    Serial.println (uint64ToStr(UserData.VFO_B_Freq, true));
  }   
}


/* -----------------------------------------------------------------------------------------------------
/ PLL, I2C, StatusLED, EEPROM and other hardware routines
/ ------------------------------------------------------------------------------------------------------
*/



boolean SwitchIsInCATMode()
{
  boolean Test = false;
  boolean TempLEDValue;
  
  TempLEDValue=LEDValue;    //Save the state of the LED Pin in a local variable 
  SetStatusLED(LOW);        //Set LED Pin to Low,
  Test = (digitalRead(PROG_Switch) == LOW);//If ProgSwitch goes low when we set LEDPin to LOW then we have a conenction between them and Swith is in CAT Mode
  SetStatusLED(TempLEDValue);        //Restore LED Pin to previous state
  return Test;
}

void SetStatusLED(boolean LEDOuput)
{
  LEDValue=LEDOuput; //Save LED status in global variable so we can remember it between functions and calls 
  digitalWrite(StatusLED,LEDOuput); //Turn ON or OFF the LED 
}

void ToggleStatusLED () //Turn ON LED if OFF and vice versa
{
  SetStatusLED(!LEDValue);
}


//Brief flash on the Status LED 'Blinks'" number of time
void LEDBlink(int Blinks)
{
  for (int i = 0; i < Blinks; i++)
  {
    SetStatusLED(LOW);   //ON
    delay (50);
    SetStatusLED(HIGH);  //OFF
    delay (50);
  }
}



//CRC calculation from Christopher Andrews : https://www.arduino.cc/en/Tutorial/EEPROMCrc
//Calculate CRC on either Factory data or Userspace data
unsigned long GetEEPROM_CRC(boolean EEPROMSpace) 
{

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;
  int Start;
  int Length;

  if (EEPROMSpace == FactorySpace)
  {
    Start = 400;
    Length = sizeof(FactoryData);
  }
  else
  {
    Start = 0;
    Length = sizeof(UserData);
  }
  for (int index = Start; index < (Start + Length) ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}


//Load FactoryData or UserSpace Data from ATMega EEPROM
bool LoadFromEPROM (boolean EEPROMSpace)
{
  int Start;
  int Length;
  unsigned long CRCFromEEPROM, CalculatedCRC;

  if (EEPROMSpace == FactorySpace) //Factory data
  {
    Start = 400;
    Length = sizeof(FactoryData);
    EEPROM.get(Start, FactoryData);                    //Load all the data from EEPROM
    CalculatedCRC = GetEEPROM_CRC(FactorySpace);       //Calculate the CRC of the data
  }
  else   //User data
  {
    Start = 0;
    Length = sizeof(UserData);
    EEPROM.get(Start, UserData);                     //Load all the data from EEPROM
    CalculatedCRC = GetEEPROM_CRC(UserSpace);          //Calculate the CRC of the data
  }
  EEPROM.get(Start + Length, CRCFromEEPROM);           //Load the saved CRC at the end of the data
  return (CRCFromEEPROM == CalculatedCRC);             //If  Stored and Calculated CRC are the same return true
}


//Save FactoryData or UserSpace Data to Arduino EEPROM
void SaveToEEPROM (boolean EEPROMSpace)
{
  int Start;
  int Length;
  unsigned long CRCFromEEPROM;
  if (EEPROMSpace == FactorySpace)
  {
    Start = 400;
    Length = sizeof(FactoryData);
    EEPROM.put(Start, FactoryData);         //Save all the Factory data to EEPROM at adress400
  }
  else //UserSpace
  {
    Start = 0;
    Length = sizeof(UserData);
    EEPROM.put(Start, UserData);          //Save all the User data to EEPROM at adress0
  }
  CRCFromEEPROM = GetEEPROM_CRC (EEPROMSpace);  //Calculate CRC on the saved data
  EEPROM.put(Start + Length, CRCFromEEPROM);    //Save the CRC after the data
  EEPROM.commit();
}


boolean DetectSi5351I2CAddress()
{
  uint8_t I2CResult;
  boolean Result;
  Si5351I2CAddress = 96; //Try with the normal adress of 96
  Wire.beginTransmission (Si5351I2CAddress);

  if (Wire.endTransmission () == 0)
  {
    //We found it
    Result = true;
  }
  else
  {
    //Serial.println("Not Detected at adress 96");
    Si5351I2CAddress = 98; //Try the alternative address of 98
    Wire.beginTransmission (Si5351I2CAddress);
    if (Wire.endTransmission () == 0)
    {
      Result = true;
    }
    else
    {
       if (!SwitchIsInCATMode ()) //the switch is not in CAT mode so we can send a ZachTek Serial API warning message 
       {
         Serial.println (F("{MIN}Hardware ERROR! No Si5351 PLL device found on the I2C buss!"));
       }  
      Result = false;
      Si5351I2CAddress = 0;
    }
  }
  return Result;
}


uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(Si5351I2CAddress);  // Start session with the Si5351PLL at I2C address
  Wire.write(reg);                           // Register number
  Wire.write(data);                          // Data to put into register
  Wire.endTransmission();
  return 0;
}

  
void SetLO()
{
  uint64_t Freq;

  if(UserData.ActiveVFO==VFO_A)
    {//VFO A is active 
    Freq=UserData.VFO_A_Freq;  
    }
  else
    {
     Freq=UserData.VFO_B_Freq;  
  } 
  if (Si5351I2CAddress == 0)
    {
    if (!SwitchIsInCATMode ()) //the switch is not in CAT mode so we can send a ZachTek Serial API warning message 
       {  
       Serial.println (F("{MIN}Hardware ERROR! No Si5351 PLL device found on the I2C buss!"));
       Debug.println (F("{MIN}Hardware ERROR! No Si5351 PLL device found on the I2C buss!"));
       }
    }
  else
    {  
    si5351aSetFrequency(Freq*4ULL);//Set LO to four times as high as it is divied down internally by four in hardware
  }
}

void SetDetector ()
{
E_Detector Detector;
   if(UserData.ActiveVFO==VFO_A)
    {//VFO A is active 
    Detector=UserData.VFO_A_Detector;  
    }
    else
    {
     Detector=UserData.VFO_B_Detector;  
    } 
  
  switch (Detector)
  {
    case USB:
      digitalWrite(SidebandSelction_IOPin, HIGH); //Set to Upper Sideband
      break;

    case LSB:
      digitalWrite(SidebandSelction_IOPin, LOW); //Set to Lower Sideband;
      break;
  } 
}


//PLL routines from Hans Summer demo code https://www.qrp-labs.com/images/uarduino/uard_demo.ino
//
// Set up specified PLL with mult, num and denom
// mult is 15..90
// num is 0..1,048,575 (0xFFFFF)
// denom is 0..1,048,575 (0xFFFFF)
//
void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1; // PLL config register P1
  uint32_t P2; // PLL config register P2
  uint32_t P3; // PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}


//PLL routines from Han Summer demo code https://www.qrp-labs.com/images/uarduino/uard_demo.ino
//
// Set up MultiSynth with integer Divider and R Divider
// R Divider is the bit value which is OR'ed onto the appropriate
// register, it is a #define in si5351a.h
//
void setupMultisynth(uint8_t synth, uint32_t Divider, uint8_t rDiv)
{
  uint32_t P1; // Synth config register P1
  uint32_t P2; // Synth config register P2
  uint32_t P3; // Synth config register P3

  P1 = 128 * Divider - 512;
  P2 = 0; // P2 = 0, P3 = 1 forces an integer value for the Divider
  P3 = 1;

  i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1, (P3 & 0x000000FF));
  i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4, (P1 & 0x000000FF));
  i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 &
                  0x000F0000) >> 16));
  i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7, (P2 & 0x000000FF));
}





// Set CLK0 output ON and to the specified frequency
// Frequency is in the range 10kHz to 150MHz and given in centiHertz (hundreds of Hertz)
// Example: si5351aSetFrequency(1000000200);
// will set output CLK0 to 10.000,002MHz
//
// This example sets up PLL A
// and MultiSynth 0
// and produces the output on CLK0
//
void si5351aSetFrequency(uint64_t frequency) //Frequency is in centiHz
{
  static uint64_t oldFreq;
  int32_t FreqChange;
  uint64_t pllFreq;
  //uint32_t xtalFreq = XTAL_FREQ;
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t Divider;
  uint8_t rDiv;


  if (frequency > 100000000ULL) { //If higher than 1MHz then set R output divider to 1
    rDiv = SI_R_DIV_1;
    Divider = 90000000000ULL / frequency;// Calculate the division ratio. 900MHz is the maximum VCO freq (expressed as deciHz)
    pllFreq = Divider * frequency; // Calculate the pllFrequency:
    mult = pllFreq / (FactoryData.RefFreq * 100UL); // Determine the multiplier to
    l = pllFreq % ( FactoryData.RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /=  FactoryData.RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }
  else // lower freq than 1MHz - use output Divider set to 128
  {
    rDiv = SI_R_DIV_128;
    //frequency = frequency * 128ULL; //Set base freq 128 times higher as we are dividing with 128 in the last output stage
    Divider = 90000000000ULL / (frequency * 128ULL);// Calculate the division ratio. 900MHz is the maximum VCO freq

    pllFreq = Divider * frequency * 128ULL; // Calculate the pllFrequency:
    //the Divider * desired output frequency
    mult = pllFreq / (FactoryData.RefFreq * 100UL); // Determine the multiplier to
    //get to the required pllFrequency
    l = pllFreq % (FactoryData.RefFreq * 100UL); // It has three parts:
    f = l; // mult is an integer that must be in the range 15..90
    f *= 1048575; // num and denom are the fractional parts, the numerator and denominator
    f /= FactoryData.RefFreq; // each is 20 bits (range 0..1048575)
    num = f; // the actual multiplier is mult + num / denom
    denom = 1048575; // For simplicity we set the denominator to the maximum 1048575
    num = num / 100;
  }


  // Set up PLL A with the calculated  multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);

  // Set up MultiSynth Divider 0, with the calculated Divider.
  // The final R division stage can divide by a power of two, from 1..128.
  // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
  // If you want to output frequencies below 1MHz, you have to use the
  // final R division stage
  setupMultisynth(SI_SYNTH_MS_0, Divider, rDiv);

  // Reset the PLL. This causes a glitch in the output. For small changes to
  // the parameters, you don't need to reset the PLL, and there is no glitch
  FreqChange = frequency - oldFreq;

  if ( abs(FreqChange) > 100000) //If changed more than 1kHz then reset PLL (completely arbitrary choosen)
  {
    i2cSendRegister(SI_PLL_RESET, 0xA0);
  }

  // Finally switch on the CLK0 output (0x4F)
  // and set the MultiSynth0 input to be PLL A
  i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
  oldFreq = frequency;
}

void SerialPrintZero()
{
  Serial.print("0");
}


uint64_t  StrTouint64_t (String InString)
{
  uint64_t y = 0;

  for (int i = 0; i < InString.length(); i++) {
    char c = InString.charAt(i);
    if (c < '0' || c > '9') break;
    y *= 10;
    y += (c - '0');
  }
  return y;
}



String  uint64ToStr (uint64_t p_InNumber, boolean p_LeadingZeros)
{
  char l_HighBuffer[7]; //6 digits + null terminator char
  char l_LowBuffer[7]; //6 digits + null terminator char
  char l_ResultBuffer [13]; //12 digits + null terminator char
  String l_ResultString = "";
  uint8_t l_Digit;

  sprintf(l_HighBuffer, "%06lu", p_InNumber / 1000000L); //Convert high part of 64bit unsigned integer to char array
  sprintf(l_LowBuffer, "%06lu", p_InNumber % 1000000L); //Convert low part of 64bit unsigned integer to char array
  l_ResultString = l_HighBuffer;
  l_ResultString = l_ResultString + l_LowBuffer; //Copy the 2 part result to a string

  if (!p_LeadingZeros) //If leading zeros should be removed
  {
    l_ResultString.toCharArray(l_ResultBuffer, 13);
    for (l_Digit = 0; l_Digit < 12; l_Digit++ )
    {
      if (l_ResultBuffer[l_Digit] == '0')
      {
        l_ResultBuffer[l_Digit] = ' '; // replace zero with a space character
      }
      else
      {
        break; //We have found all the leading Zeros, exit loop
      }
    }
    l_ResultString = l_ResultBuffer;
    l_ResultString.trim();//Remove all leading spaces
  }
  return l_ResultString;
}
