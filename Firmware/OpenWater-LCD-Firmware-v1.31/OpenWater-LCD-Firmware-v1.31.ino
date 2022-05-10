
/*************************************************************
 * OpenWater Turbidity Meter Code
 * Sheldon Stokes
 * Feb 22, 2019
 * 
 * Added serial code for bluno BLE.  The bluno acts as a wireless serial port
 * and the way I'm commuicating is via the DFB1 service
 * 
 * Byte                 Function
 * 0x00                 Collect Data
 * 0x01                 Read last measurement Mean
 * 0x02                 Read last measurement StDev
 * 0x03                 Read num averages
 * 0x04                 Read integration time
 * 0x05                 Read Slope
 * 0x06                 Read Intercept
 * 0x07                 Read mean counts
 * 0x08                 Read stdev counts
 * 0x09                 Read all above values in one sring
 * 0x10                 Read calibration date
 * 0x11                 Write calibration date
 * 0x12                 start Calibration
 * 0x13                 Read Cal Date Human Readable
 * 0x14                 Read Busy
 * 
 * 
 * In my BLE program you send commands as 01:0D for read last measurement mean
 * 
 *************************************************************/


// include the library code:
#include <EEPROM.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

//********************Defines *************************
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7


// ********************* EEPROM Stuff ******************

#define calSlopeAddress     0
#define calIntAddress       4
#define numAvesAddress      8
#define intTimeAddress      12
#define lowConcAddress      16
#define highConcAddress     24
#define debugActiveAddress  32
#define calDateAddress      36


//******************** variables **********************

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();





#define RUNSTATE 0
#define CALSTATE 1
String versionNum = "1.31";

int det1Pin = 2;
int ledPin = 12;
int ledOnDelay = 2000;

int state,exitState;    

float calSlope, calInt;
int numDelaysForAve, debugActive;
float intTime, lowAct, highAct;
long calDate, bleDate;
long dStr1,dStr2,dStr3,dStr4;

float lowCnt, highCnt;
float stdDv, mean, stDevAcc,meanConc, stDevConc;

float lastReading, lastStDev;
volatile int takeReading = 0;
volatile int isBusy = 0;

volatile long cnt, dark,sig;

uint8_t buttons;

 
String str = "";
int len;


// date related variables
char out[25];
int calDay,calMonth,calYear,calHour,calMinute,calSecond;

void setup() {

  // load values from EEPROM
  EEPROM.get( calSlopeAddress, calSlope );
  EEPROM.get( calIntAddress , calInt );
  EEPROM.get( numAvesAddress , numDelaysForAve );
  EEPROM.get( intTimeAddress , intTime );
  EEPROM.get( lowConcAddress , lowAct );
  EEPROM.get( highConcAddress , highAct );
  EEPROM.get( debugActiveAddress, debugActive );
  EEPROM.get( calDateAddress, calDate);

  // trap blank EEPROM values
  if (isnan(calSlope)) calSlope= 1.0;
  if (isnan(calInt)) calInt = 0.0;  
  if (isnan(numDelaysForAve)) numDelaysForAve = 3;
  if (numDelaysForAve < 0) numDelaysForAve = 3;
  if (isnan(intTime)) intTime = 2.0; 
  if (isnan(lowAct)) lowAct  = 0.1; 
  if (isnan(highAct)) highAct = 1.0; 
  if (isnan(debugActive)) debugActive = 1;
  if (isnan(calDate)) calDate = 1583445767;

  Serial.begin(115200);   //this will start the serial port to talk through BLE on the bluno

  pinMode(LED_BUILTIN, OUTPUT);


  // setup IO pins
  pinMode(det1Pin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);

  // pat ourselves on the back
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" OpenWater ");
  lcd.print(versionNum);



  // set timer overflow interrupt for Bluetooth coms so it doesn't block when collecting data

  //set timer1 interrupt at 10Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 5hz increments
  OCR1A = 1563;// = (16*10^6) / (10*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);




 // state = RUNSTATE;
  attachInterrupt(digitalPinToInterrupt(det1Pin), pulseISR, FALLING);

 
}  //setup

void loop() {


  buttons = lcd.readButtons();


  if ((buttons)||(takeReading)) {
      lcd.clear();
      lcd.setCursor(0,0);

      if ((buttons & BUTTON_SELECT)||(takeReading)) {       
          
          lcd.clear();
          while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released
          
          while (!(buttons & BUTTON_SELECT)&&(!takeReading)){


           buttons = lcd.readButtons();

           if (buttons & BUTTON_RIGHT) {
              exitState = CALSTATE;
              calDate++;      // add one second to existing calibration date to show a new cal happened
           }
           if (buttons & BUTTON_LEFT) exitState = RUNSTATE;
        
            
            lcd.setCursor(0,0);
            lcd.print(" Collect Data");
            lcd.setCursor(0,1);
            lcd.print(" Calibrate ");

            //clear edge
            lcd.setCursor(14,(exitState+1) & 0x1);
            lcd.print("  ");
            lcd.setCursor(14,exitState);
            lcd.print("<=");
            
          }  //while

          if (takeReading){
           // exitState = RUNSTATE;
            takeReading = 0;
          }

/******************************************************************************************
 **************           Cal State         ***********************************************
 ******************************************************************************************/

          if (exitState == CALSTATE){ 


            //************ calibration date routine *********************
            adjustCalDate();

            lcd.clear();
            lcd.print("Print Counts:"); 

            while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
            while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
              delay(250);
              buttons = lcd.readButtons();
              lcd.setCursor(0,1);
              if (debugActive == 1)  lcd.print("Yes");
              else lcd.print("No");
              lcd.print("   ");
              if(buttons & BUTTON_RIGHT) debugActive = 1;
              if(buttons & BUTTON_LEFT) debugActive = 0;
            }
            EEPROM.put(debugActiveAddress,debugActive);



/**********************************************/


            lcd.clear();
            lcd.print("Num Averages:"); 


            while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
            while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
              delay(250);
              buttons = lcd.readButtons();
              lcd.setCursor(0,1);
              lcd.print(numDelaysForAve);
              lcd.print("   ");
              if(buttons & BUTTON_RIGHT) numDelaysForAve += 1;
              if(buttons & BUTTON_LEFT) numDelaysForAve -= 1;
            }
            EEPROM.put(numAvesAddress,numDelaysForAve);


/**********************************************/

  
            lcd.clear();
            lcd.print("Int Time:"); 


            while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
            while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
              delay(100);
              buttons = lcd.readButtons();
              lcd.setCursor(0,1);
              lcd.print(intTime);
              lcd.print("   ");
              if(buttons & BUTTON_RIGHT) intTime += 0.1;
              if(buttons & BUTTON_LEFT) intTime -= 0.1;
            }
            EEPROM.put( intTimeAddress , intTime );


/**********************************************/
          
            
            lcd.clear();
            lcd.print("1st Soln Conc:"); 
  
          
            while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
            while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
              delay(100);
              buttons = lcd.readButtons();
              lcd.setCursor(0,1);
              lcd.print(lowAct);
              if(buttons & BUTTON_RIGHT) lowAct += 0.1;
              if(buttons & BUTTON_LEFT) lowAct -= 0.1;
            }
            EEPROM.put( lowConcAddress , lowAct );

            
            //****************** collect data here ****************************
            collectData();
            lowCnt = sig-dark;
            

/**********************************************/



            lcd.clear();
            lcd.print("2nd Soln Conc:");   
          
            while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
            while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
              delay(100);
              buttons = lcd.readButtons();
              lcd.setCursor(0,1);
              lcd.print(highAct);
              if(buttons & BUTTON_RIGHT) highAct += 0.1;
              if(buttons & BUTTON_LEFT) highAct -= 0.1;
            }
            EEPROM.put( highConcAddress , highAct );
            
            //****************** collect data here ****************************
            collectData();
            highCnt = sig-dark;




            //****************** calculate coefficients ***********************

            calSlope = (highAct - lowAct) / (highCnt - lowCnt);
            calInt = lowAct - (calSlope * lowCnt);
            
            EEPROM.put( calSlopeAddress, calSlope );
            EEPROM.put( calIntAddress , calInt );


          // add cal date
          //  EEPROM.put( calDateAddress, calDate);
          putCalDate();
            
                  
          } //calcase


/******************************************************************************************
 **************           Run State         ***********************************************
 ******************************************************************************************/


          if (exitState == RUNSTATE){  

            isBusy = 1;
            collectData();

        // calculate the mean and stDev concentration


//            stDevAcc = 0;
//            meanConc = ((sig-dark) * calSlope) + calInt;
//            for (int i = 0; i < numDelaysForAve; i++)
//            {
//              stDevAcc += sq((((readings[i]-dark) * calSlope) + calInt) - meanConc) ; 
//            }
//            stDevConc = sqrt(stdevAcc/numDelaysForAve);

            lcd.clear();
            lcd.print("Reading:");
            lcd.setCursor(0,1);            
            lcd.print(((sig-dark) * calSlope) + calInt);

//            Serial.print(sig);
//            Serial.print(",");
//            Serial.print(mean);
//            Serial.print(",");
//            Serial.print(dark);
//            Serial.print(",");
//            Serial.print(((sig-dark) * calSlope) + calInt);           
//            
//            lcd.print(meanConc);
            lcd.setCursor(8,1);
            lcd.print(stdDv*calSlope);

            lastReading = ((sig-dark) * calSlope) + calInt;
            lastStDev = stdDv*calSlope;

            isBusy = 0;
            // convert to conc

            
          }  //runstate




      }
    }


}  //loop



void pulseISR() {
  cnt++;
}

float collectData()
{

  float acc = 0;

  float readings[numDelaysForAve];
  float stAcc;

 lcd.clear();
 lcd.print("Dark Read...");

 // collect dark data
  digitalWrite(ledPin, LOW); 
  cnt = 0;
  interrupts();
  cnt = 0;
  delay(intTime*1000);
  noInterrupts();
  dark = cnt;
  interrupts();

  if (debugActive == 1){
      lcd.clear();
      lcd.print("darkCnt:");
      lcd.setCursor(0,1);
      lcd.print(dark);
      delay(2000);
  }

  lcd.clear();
  lcd.print("Warming...");
  lcd.setCursor(0,1);
  lcd.print("*** Laser ON ***");  
  digitalWrite(ledPin, HIGH);  
  delay(ledOnDelay);



// collect light data

      for (int i = 0; i < numDelaysForAve; i++)
    {

       lcd.clear();
       lcd.print("Ave: ");
       lcd.print(i+1);
       lcd.print(" of ");
       lcd.print(numDelaysForAve);
       lcd.setCursor(0,1);
       lcd.print("*** Laser ON ***");         
       
       cnt = 0;
       delay(intTime*1000); 
       noInterrupts();
       acc += cnt; 
       readings[i] = cnt;
       interrupts();

       if (debugActive == 1){
           lcd.clear();
           lcd.print("Counts:");
           lcd.setCursor(0,1);
           lcd.print(cnt);
           delay(2000);
       }


    } // for loop
   lcd.clear();
   lcd.print("*** Done ***");
   digitalWrite(ledPin, LOW);  

    mean = (acc/numDelaysForAve);
    sig = long(mean);                   // this used to be an int typecast and caused an overflow
    stAcc = 0;
    for (int i = 0; i < numDelaysForAve; i++)
    {
      stAcc += sq(readings[i] - mean);
    }
    stdDv = sqrt(stAcc/numDelaysForAve);
    
} //end collect Data


//void bleCheck()
ISR(TIMER1_COMPA_vect)
{
  /* add bluetooth control here
 * 0x00                 Collect Data
 * 0x01                 Read last measurement Mean
 * 0x02                 Read last measurement StDev
 * 0x03                 Read num averages
 * 0x04                 Read integration time
 * 0x05                 Read Slope
 * 0x06                 Read Intercept
 * 0x07                 Read mean counts
 * 0x08                 Read stdev counts
 * 0x09                 Read everything
 * 0x10                 Read Cal Date
 * 0x11                 Write calibration date
 * 0x12                 Start Calibration
 * 0x13                 Read Cal Date Human Readable
 * 0x14                 Read Busy
 * 
  */

    len  = getCmd();
    if (len > 0)
  {
   
   // byte cmd = Serial.read();
    switch (str[0]){
      case 0x00:
        exitState = RUNSTATE;
        takeReading = 1;
        break;
      case 0x01:
        Serial.println(lastReading, 8);
        break;
      case 0x02:
        Serial.println(lastStDev, 8);
        break;
      case 0x03:
        Serial.println(numDelaysForAve);
        break;
      case 0x04:
        Serial.println(intTime);
        break;
      case 0x05:
        Serial.println(calSlope, 8);
        break;
      case 0x06:
        Serial.println(calInt, 8);
        break;       
      case 0x07:
        Serial.println(mean, 8);
        break;
      case 0x08:
        Serial.println(stdDv, 8);
        break;    
      case 0x09:
        Serial.print(lastReading, 8);
        Serial.print(",");
        Serial.print(lastStDev, 8);
        Serial.print(",");
        Serial.print(numDelaysForAve);
        Serial.print(",");
        Serial.print(intTime);
        Serial.print(",");
        Serial.print(calSlope, 8);
        Serial.print(",");
        Serial.print(calInt, 8);
        Serial.print(",");
        Serial.print(mean, 8);
        Serial.print(",");
        Serial.print(stdDv, 8);
        Serial.print(",");
        Serial.println(calDate);
        
        break;

    case 0x10:

        Serial.println(calDate); 
        break;
 
    case 0x11:
      
        calDate = str.substring(1).toInt();
        EEPROM.put( calDateAddress, calDate);
 //       Serial.println(calDate);
        break;
        
    case 0x12:
        bleDate = str.substring(1).toInt();
        if (bleDate < calDate) bleDate = calDate++;   // check for a valid new date.  a minimal check
        calDate = bleDate;
        exitState = CALSTATE;
        takeReading = 1;
        break;

   case 0x13:


        calDay = day(calDate);
        calMonth = month(calDate);
        calYear = year(calDate); 
        calHour = hour(calDate);  
        calMinute = minute(calDate); 
        calSecond = second(calDate);   

        sprintf(out,"%02d/%02d/%04d  %02d:%02d:%02d",calDay,calMonth,calYear,calHour,calMinute,calSecond);
     
        Serial.print(out);
        break;   

  case 0x14:
        Serial.print(isBusy);
        break;
        
    }
    str = "";
  }
}


/*******************************************************************
 *****                     General Routines                    *****
 *******************************************************************/

int getCmd(){  
  
  if (Serial.available())  {    
    char c = Serial.read();  //gets one byte from serial buffer
      str+=c;
    while (c != 13){
      if (Serial.available()){
        c = Serial.read();
        str+=c;
      }
    }
  }

return str.length();
}


/*******************************************************************
 *****                    Cal Date Routines                    *****
 *******************************************************************/

void getCalDate(){

   EEPROM.get( calDateAddress, calDate);
   calDay = day(calDate);
   calMonth = month(calDate);
   calYear = year(calDate); 
   calHour = hour(calDate);  
   calMinute = minute(calDate); 
   calSecond = second(calDate);  
   return;
}

void putCalDate(){

  setTime(calHour, calMinute, calSecond, calDay, calMonth, calYear);
  calDate = now();
  EEPROM.put( calDateAddress, calDate);
  return;
}

void adjustCalDate(){

  getCalDate();
  sprintf(out,"%02d/%02d/%04d %02d:%02d",calMonth,calDay,calYear,calHour,calMinute);
     
  lcd.clear();
  lcd.print("Cal Date:"); 
  lcd.setCursor(0,1);
  lcd.print(out);


  // ***************** Month ********************** 
  while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
    while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
       buttons = lcd.readButtons();
       lcd.setCursor(0,1);
       lcd.print("  ");
       delay(150);
       lcd.setCursor(0,1);
       sprintf(out,"%02d",calMonth);
       lcd.print(out);
       delay(150);
          
       if(buttons & BUTTON_RIGHT) calMonth++;
       if(calMonth > 12) calMonth = 1; 
       if(buttons & BUTTON_LEFT) calMonth--;
       if(calMonth < 1) calMonth = 12;             
    }

    
   // ***************** Day ********************** 
  while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
    while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
       buttons = lcd.readButtons();

       lcd.setCursor(3,1);
       lcd.print("  ");
       delay(150);
       lcd.setCursor(3,1);
       sprintf(out,"%02d",calDay);
       lcd.print(out);
       delay(150);
          
       if(buttons & BUTTON_RIGHT) calDay++;
       if(calDay > 31) calDay = 1;                  // fix days in month, this is yucky
       if(buttons & BUTTON_LEFT) calDay--;
       if(calDay < 1) calDay = 31;             
    }

   // ***************** year ********************** 
  while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
    while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
       buttons = lcd.readButtons();

       lcd.setCursor(6,1);
       lcd.print("    ");
       delay(150);
       lcd.setCursor(6,1);
       sprintf(out,"%04d",calYear);
       lcd.print(out);
       delay(150);
          
       if(buttons & BUTTON_RIGHT) calYear++;
       if(buttons & BUTTON_LEFT) calYear--;
       if(calYear < 2020) calYear = 2020;             
    }
  // ***************** Hour ********************** 
  while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
    while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
       buttons = lcd.readButtons();
       lcd.setCursor(11,1);
       lcd.print("  ");
       delay(150);
       lcd.setCursor(11,1);
       sprintf(out,"%02d",calHour);
       lcd.print(out);
       delay(150);
          
       if(buttons & BUTTON_RIGHT) calHour++;
       if(calHour > 23) calHour = 0; 
       if(buttons & BUTTON_LEFT) calHour--;
       if(calHour < 0) calHour = 23;             
    }

  // ***************** Minute ********************** 
  while (buttons & BUTTON_SELECT) buttons = lcd.readButtons();    //wait for button to be released           
    while (!(buttons & BUTTON_SELECT)){     // stay in loop until select button is pressed
       buttons = lcd.readButtons();
       lcd.setCursor(14,1);
       lcd.print("  ");
       delay(150);
       lcd.setCursor(14,1);
       sprintf(out,"%02d",calMinute);
       lcd.print(out);
       delay(150);
          
       if(buttons & BUTTON_RIGHT) calMinute++;
       if(calMinute > 59) calMinute = 0; 
       if(buttons & BUTTON_LEFT) calMinute--;
       if(calMinute < 0) calMinute = 59;             
    }  

    calSecond = 0;
      
  return;
}
