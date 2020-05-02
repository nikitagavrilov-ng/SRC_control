//NG industries by Nikita Gavrilov, 01.04.2020, email: nikitagavrilov.ng@gmail.com
// Setpoint control works OK
//Torque and Speed control does not work because of PID and sensors
#include "SRC_control.h"

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

//outputs
#define SPEEDPIN 5
#define TURBOPIN 6
#define BREAKPIN 9
//10-13 pins can be used for SPI


int SetHamMin=440, SetHamMax=540,HammerPos=SetHamMin;    // set this values at the middle so they are tunable to both sides when variable resister will get older
Controller General=Controller(MINTORQUE, MAXTORQUE, CHOKE, SetHamMin, SetHamMax);   
SetOutputs Outputs=SetOutputs();
Measurements VoltageSensor=Measurements(4);
Measurements CurrentSensor=Measurements(2);
byte mode, type;   //0-speed control; 1-torque control; 2-setpoint generation  // 0 tested OK, 1 tested OK, 2 not tested 
 
int Control ();
int ParameterAssignment();
void setup() {
    // initialise inputs and outputs   
    Serial.begin(9600);
    
    pinMode(HAMPOS, INPUT);  
    pinMode(MINTORQUE, INPUT);      //min speed
    pinMode(MAXTORQUE, INPUT);      //max speed
    pinMode(BREAKVAL, INPUT);       //break   
    pinMode(CHOKE, INPUT);  
    pinMode(SPEEDPIN, OUTPUT);
    pinMode(BREAKPIN, OUTPUT);
    pinMode(TURBOPIN, OUTPUT);
    pinMode(SET1, INPUT);
    pinMode(SET2, INPUT);
    pinMode(SET3, INPUT);
    pinMode(SET4, INPUT);
    


    // initialising timer interrupt for timer1 (overflow)
    cli();                          //deactivate interrupt
    TCCR1A=0;                       // clear timer variables
    TCCR1B=0;

    TIMSK1= (1<<TOIE1);             // start timer
    TCCR1B |= (1 << CS11);          // set timer preselector to 1/8
    //TCCR1B |= (1 << CS10);
    sei();                          //activate interrupt

    ParameterAssignment();          // assignment of parameters for controller
    }
  
  void loop() {
      HammerPos=limit(analogRead(HAMPOS),SetHamMin,SetHamMax);            // get speed control hammer position and scale it according to actual mechanics
    //----------------------Calculation of set value according to sensor data---------------------------
    int RawOut=Control();                                                 //RawOut must be 0...255 value if min and max are set right      
    
    //-----------------------------------------Set outputs-------------------------------------------  
                                                                          // Raw value is an integer 0..260, 0- break, 1..255- speed control, 256..260- turbo
    Outputs.TransistorControl(RawOut, SPEEDPIN);                          //control of using Transistor
    Outputs.TurboControl(RawOut, TURBOPIN);                               //control of using Relay - Turbo mode  
    //Outputs.CommitBreakClassic(RawOut, BREAKPIN, BREAKVAL);
    Outputs.CommitBreak(RawOut, BREAKPIN, BREAKVAL);                      //control of using Break
  
  
  //Serial.println(" end ");
  //delay(1000); // delete it in operation or scale to different value
}

//--------------------------------end of main----------------------------------------------------------


int Control (){
    //use linear, square root
    int Setpoint, current, voltage;                                                             // set some local variables
    
    switch (mode) {                                                                             // mode of control type, 0- linear setpoint, 1- torque control, 2- speed control, 3- square root setpoint
            case 0:                                                                             //Setpoint setting;                 
                  General.CalculateSetpointClassic(HammerPos);                                  //Generate setpoint linear
                  Setpoint=General.Finalise(HammerPos);                                         //finalise setpoint in terms of break and turbo control
            break;          
            case 1:                                                                             // Torque control;    
                  current=ScaleAnalog(CurrentSensor.RunningAverage(CPIN,LOW),512,0,0,3000);     //current = ScaleAnalog(analogRead(CPIN),512,1023,0,30);          
                  General.CalculateTorque(HammerPos,current);       
                    Setpoint=General.Finalise(HammerPos);                                       //finalise setpoint in terms of break and turbo control
              break;        
            case 2:                                                                             // Speed control;
                  current=ScaleAnalog(CurrentSensor.RunningAverage(CPIN,LOW),512,0,0,3000); 
                  voltage=ScaleAnalog(VoltageSensor.RunningAverage(VPIN,LOW),0,1023,0,3700);
                  Setpoint=General.CalculateSpeed(HammerPos,current, voltage);     
                  Setpoint=General.Finalise(HammerPos);                                         //finalise setpoint in terms of break and turbo control
            break;
            
            case 3:
                  General.CalculateSetpoint(HammerPos);                                         //Generate setpoint as square root function
                  Setpoint=General.Finalise(HammerPos);                                         //finalise setpoint in terms of break and turbo control
            break;
                }


  return Setpoint; //0...260
  }



ISR(TIMER1_OVF_vect)
{  
  ParameterAssignment();          // assignment of parameters for controller
}

int ParameterAssignment(){
  mode=digitalRead(SET1)<<0|digitalRead(SET2)<<1;    // mode of control type, 0- linear setpoint, 1- torque control, 2- speed control, 3- square root setpoint
  type=digitalRead(SET3)<<0|digitalRead(SET4)<<1;    // type of min and max setting range, 0-slow, 1-middle, 2-fast, 3- super fast
  General.ChangeType(type); //it is not the reason
      
      
       switch (mode) {     // it is the cause of unexpected output deviations     
            case 0:    // Setpoint setting;
                  General.SetpointParameters();
            break;          
            case 1:    // Torque setting;    
                  General.TorqueParameters();
            break;        
            case 2:    // Speed setting;
                 General.SpeedParameters();
            break;
            case 3:    // Speed control;
                 General.SetpointParameters();
            break;
       }
       }
//--------------------------------------------------------------------------------------
