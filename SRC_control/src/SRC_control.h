//NG industries by Nikita Gavrilov, 01.04.2020, email: nikitagavrilov.ng@gmail.com

#ifndef SRC_control_h
#define SRC_control_h


int limit (int value, int min, int max);                        // return integer of limited "value" from "min" to "max"
int ScaleAnalog(int X, int Xmin, int Xmax, int Ymin, int Ymax); // return an integer linearly scaled value of X between 2 points (Xmin, Xmax, Ymin, Ymax)
int ScaleSquare(int X, int Xmin, int Xmax, int Ymin, int Ymax); // return an integer scaled as sqrt(x) value of X between 2 points (Xmin, Xmax, Ymin, Ymax)

//int Control (int current, int voltage, int BreakVal, int TurboVal);

class Measurements
{
  
  public:
    Measurements( int RACount);
    int RunningAverage(int PINnumber, bool data);
  private:  
    long int NextRAValue, RAValue;
    int RACount;
    int RABuffer[32];

};


class Controller
{
  public:
    Controller(int MinPin, int MaxPin, int ChokePin, int SetHamMin, int SetHamMax);
    void ChangeType(char type);
    int CalculateSetpointClassic(int HammerPos);
    int CalculateSetpoint(int HammerPos);
    int SetpointParameters(void);
    int CalculateTorque(int HammerPos, int current);
    int TorqueParameters(void);
    int CalculateSpeed(int HammerPos, int current, int voltage);
    int SpeedParameters(void);
    void ChangeR(int newR);
    int Finalise(int HammerPos);   
  private:
    int type_I;
    int prevE;
    long int count;
    int MinPin_I;
    int MaxPin_I;
    int SetHamMin_I;
    int SetHamMax_I;
    Measurements MinTorqueSensor=Measurements(3);
    Measurements MaxTorqueSensor=Measurements(3);
    Measurements SensorFunctionOutput=Measurements(8);
    int MinTorque;
    int MaxTorque;
    int Setpoint, ChokePin_I;
    int TorqueID,SpeedID;
    int R=4;       // 0.4 Ohm
    //double Input, Output, Setvalue;
    //PID* TorquePID;
};

class SetOutputs
{
    public:
      SetOutputs(void);
      int CommitBreak(int Setpoint, char pin, char brValpin);
      int CommitBreakClassic(int Setpoint, char pin, char brValpin);
	    int TransistorControl (int Setpoint, char pin);
	    int TurboControl(int Setpoint, char pin);
    private:
      unsigned long breaksetpoint;
      int LastSetPoint;
      int SpeedBreak;
};



#endif
