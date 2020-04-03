//NG industries by Nikita Gavrilov, 01.04.2020, email: nikitagavrilov.ng@gmail.com

#include "SRC_control.h"


//outputs
#define SPEEDPIN 5
#define TURBOPIN 6
#define BREAKPIN 9
//10-13 pins can be used for SPI

// analog inputs
#define HAMPOS A1
#define VPIN A3
#define CPIN A4
#define MINTORQUE A5
#define MAXTORQUE A6
#define BREAKVAL A7

// digital imputs
#define CHOKE A2
#define SET4 2
#define SET3 3
#define SET2 7
#define SET1 8


int SetHamMin=470, SetHamMax=600,HammerPos=SetHamMin;    // set this values at the middle so they are tunable to both sides when variable resister will get older
Controller General=Controller();   
SetOutputs Outputs=SetOutputs();
Measurements Sensor=Measurements(32);
void setup() {
    // initialise inputs and outputs   
    Serial.begin(9600);
    
    pinMode(HAMPOS, INPUT);  
    pinMode(MINTORQUE, INPUT);   //min speed
    pinMode(MAXTORQUE, INPUT);   //max speed
    pinMode(BREAKVAL, INPUT);   //break   
    pinMode(CHOKE, INPUT);  
    pinMode(SPEEDPIN, OUTPUT);
    pinMode(BREAKPIN, OUTPUT);
    pinMode(TURBOPIN, OUTPUT);
    pinMode(SET1, INPUT);
    pinMode(SET2, INPUT);
    pinMode(SET3, INPUT);
    pinMode(SET4, INPUT);
    }
  
  void loop() {

    // initialise input variables

    HammerPos=analogRead(HAMPOS);
    HammerPos=limit(HammerPos,SetHamMin,SetHamMax);

    //----------------------Calculation of set value according to sensor data---------------------------
    int RawOut=Control(General,Sensor.ReadCurrent(CPIN),Sensor.ReadVoltage(VPIN));  //RawOut must be 0...255 value if min and max are set right      
    
    //-----------------------------------------Set outputs-------------------------------------------  
    // Raw value is an integer 0..260, 0- break, 1..255- speed control, 256..260- turbo
    Outputs.TransistorControl(RawOut, SPEEDPIN); //control of using Transistor
    Outputs.TurboControl(RawOut, TURBOPIN); //control of using Relay - Turbo mode  
    //BreakAct.CommitClassic(RawOut, BREAKPIN, BREAKVAL);
    Outputs.CommitBreak(RawOut, BREAKPIN, BREAKVAL);
  
    //delay(100); // delete it in operation or scale to different value
}

//--------------------------------end of main----------------------------------------------------------


int Control (Controller General, int current, int voltage){
    //use linear, exp, square, root, 2 order functions
    // set vars
    int Setpoint;
    byte mode=digitalRead(SET1)<<0|digitalRead(SET2)<<1;     //0-speed control; 1-torque control; 2-setpoint generation  // 0 tested OK, 1 tested OK, 2 not tested 
    byte type=digitalRead(SET3)<<0|digitalRead(SET4)<<1;    // type of min and max setting range, 0-slow, 1-middle, 2-fast

             
    General.GetParameters(type,MINTORQUE, MAXTORQUE, CHOKE, SetHamMin,SetHamMax);

    switch (mode) {                 
            case 0:    // Setpoint setting;
                  General.CalculateSetpoint(HammerPos);
                  Setpoint=General.Finalise();
            break;           
            case 1:    // Torque control;
                  General.CalculateTorque(HammerPos,current, voltage);       
                  Setpoint=General.Finalise();
            break;        
            case 2:    // Speed control;
                  Setpoint=General.CalculateSpeed(HammerPos,current, voltage);     
                  Setpoint=General.Finalise();
            break;
            case 4:
                Serial.print("Here you can write your own function");
            break;
                }


  return Setpoint; //0...260
  }

//--------------------------------------------------------------------------------------
