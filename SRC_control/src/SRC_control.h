//NG industries by Nikita Gavrilov, 01.04.2020, email: nikitagavrilov.ng@gmail.com

#ifndef SRC_control_h
#define SRC_control_h
#include <PID_v1.h>             // add PID library


int limit (int value, int min, int max);                        // return integer of limited "value" from "min" to "max"
int ScaleAnalog(int X, int Xmin, int Xmax, int Ymin, int Ymax); // return an integer linearly scaled value of X between 2 points (Xmin, Xmax, Ymin, Ymax)


//int Control (int current, int voltage, int BreakVal, int TurboVal);




class Controller	//Class Controller deffines 3 control algorithms, that can be used in main cycle
{
  public:
    Controller(void);	// initialise class
    void GetParameters(char type, int MinPin, int MaxPin, int ChokePin, int SetHamMin, int SetHamMax);
	// Initialise required parameters of the class:
	// type of min and max setting range, 0-slow, 1-middle, 2-fast
	//MinPin- PIN that is used for setting minimal speed or torque
	//MaxPin- PIN that is used for setting maximal speed or torque
	//ChokePin- PIN that is used for changing Choke and turbo mode
	//SetHamMin- setting of minimal hammer position
	//SetHamMax- setting of maximal hammer position
    int CalculateSetpoint(int HammerPos);							// Calculate Setpoint using algorithm of basic setpoint generation according to actual hammer position
    int CalculateTorque(int HammerPos, int current);				// Calculate Setpoint using Torque control angorithm that use current value and actual hammer position
    int CalculateSpeed(int HammerPos, int current, int voltage);	// Calculate Setpoint using Speed control angorithm that use current and voltage value and actual hammer position
    void ChangeR(int newR);											// Change Armature resistance value, that is used in Torque and  Speed control algorithms. Default is 4 (i.e. 0.4Ohm)
    int Finalise(void);												// Finalise calculations and return Setpoint Value
    int type_I;														// type of min and max setting range, 0-slow, 1-middle, 2-fast
  // library-accessible "private" interface
  private:
    

    int MinPin_I;
    int MaxPin_I;
    int HammerPos_I, dHammerPos, prevHammerPos, prevdHammerPos;
    int SetHamMin_I;
    int SetHamMax_I;
    int MinTorque;
    int MaxTorque;
    int Setpoint, ChokePin_I;
    int TorqueID,SpeedID;
    int R=4;       // 0.4 Ohm
    double Input, Output, Setvalue;			// Variables, that are used in PID
    PID* TorquePID;							//PID controller
    //PID TorquePID;
        //PID myPID;
};

class SetOutputs	//Class SetOutputs defines Break, Transisor and Turbo output values according to Setpoint  value (0...260)
    // Setpoint is an integer 0..260, 0- break, 1..255- speed control, 256..260- turbo
{
    public:
      SetOutputs(void);													// initialise class
      int CommitBreak(int Setpoint, char pin, char brValpin);			//Use break transistor on "pin" if Setpoint is set to 0. brValpin defines break settings pin. This algorithm is modified CommitBreakClassic and use recuperative and dynamic break
      int CommitBreakClassic(int Setpoint, char pin, char brValpin);	//Use break transistor on "pin" if Setpoint is set to 0. brValpin defines break settings pin. This algorithm use only dynamic break
	  int TransistorControl (int Setpoint, char pin);					//Use speed contron transistor on "pin" if Setpoint is 1..255. If Setpoint is 0 then 0 is on transistor. If setpoint is 256..260- then transistor is set to 255
	  int TurboControl(int Setpoint, char pin);							//Use Turbo relay on "pin" if Setpoint is 256..260. If Setpoint is 0..255 relay is off.
    private:
      unsigned long breaksetpoint;
      int LastSetPoint;
      int SpeedBreak;
};


class Measurements				//Class Measurements defines current and voltage measurement functions with running average filtration.	
{
	
	public:
	  Measurements( int RACount);											// initialise class	with RACount-running average period.
		int ReadCurrent(int PIN);											// Function that is used for reading current
		int ReadVoltage(int PIN);											// Function that is used for reading voltage
	private:	
		int NextRACurrent, RACurrentValue, NextRAVoltage, RAVoltageValue;
		int RACount;
		int RACurrentBuffer[],RAVoltageBuffer[];
		int RunningAverageCurrent(int PINnumber);							//Calculate Running average for current
		int RunningAverageVoltage(int PINnumber);							//Calculate Running average for voltage
};

#endif
