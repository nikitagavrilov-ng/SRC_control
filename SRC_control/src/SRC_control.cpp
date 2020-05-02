//NG industries by Nikita Gavrilov, 01.04.2020, email: nikitagavrilov.ng@gmail.com

#include "SRC_control.h"
#include "Arduino.h"

    // if dirivative is hight than change to fast PID settings
    //not in use if break is on then change to slow PID
    /*            if (HammerPos<(SetHamMin+2)) 
                  myPID.SetTunings(0.001,0,0);
      */   


Controller::Controller(int MinPin, int MaxPin, int ChokePin, int SetHamMin, int SetHamMax){
    prevE=0;
    type_I= 0;
    MinPin_I=MinPin;
    MaxPin_I=MaxPin;
    ChokePin_I=ChokePin;
    SetHamMin_I=SetHamMin;
    SetHamMax_I=SetHamMax;        
    count=0;
    }

void Controller::ChangeType(char type){
    type_I= type;
}


void Controller::ChangeR(int newR){
  R=newR;
  }

  
int Controller::SetpointParameters(void){
   switch (type_I) {
      case 0:    // up to 1 Amp                   
            
          MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,60); //scale min speed inputs to outputs
          MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,100,255); //scale min speed inputs to outputs, keep in the mind that 9-th channel is 0...255  
    
      break;
      case 1: // 1..6 Amp
          MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,100); //scale min speed inputs to outputs
          MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,150,255); //scale min speed inputs to outputs, keep in the mind that 9-th channel is 0...255  
      break;
      case 2: // 6...15Amp
          MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,140); //scale min speed inputs to outputs
          MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,200,255); //scale min speed inputs to outputs, keep in the mind that 9-th channel is 0...255  
      break;  
      case 3: // 6...15Amp
          MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,180); //scale min speed inputs to outputs
          MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,220,255); //scale min speed inputs to outputs, keep in the mind that 9-th channel is 0...255  
      break;  
    }
}


int Controller::CalculateSetpointClassic(int HammerPos){
    Setpoint= ScaleAnalog(int(HammerPos),SetHamMin_I,SetHamMax_I,MinTorque, MaxTorque); // Scale Speed Hammer position         
}


int Controller::CalculateSetpoint(int HammerPos){
    Setpoint= ScaleSquare(int(HammerPos),SetHamMin_I,SetHamMax_I,MinTorque, MaxTorque); // Scale Speed Hammer position                   
}





int Controller::TorqueParameters(void){ 
   switch (type_I) {
                case 0:    // up to 1 Amp                   
                      MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,100); //scale min torque inputs to outputs MinPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                      MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,200,400); //scale min torque inputs to outputs MaxPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                break;
                case 1: // 1..6 Amp
                      MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,800); //scale min torque inputs to outputs MinPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                      MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,1200,3500); //scale min torque inputs to outputs MaxPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                break;
                case 2: // 6...15Amp
                      MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,1500); //scale min torque inputs to outputs MinPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                      MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,3000,6000); //scale min torque inputs to outputs MaxPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                break;  
                case 3: // 6...15Amp
                      MinTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,0,2500); //scale min torque inputs to outputs MinPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                      MaxTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,4000,8000); //scale min torque inputs to outputs MaxPin_I, LOW limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                break;  
          }
}


int Controller::CalculateTorque(int HammerPos, int current){      
    TorqueID=current*R;//ScaleAnalog(,0,1200,MinTorque,MaxPin_I, LOW); //maxCurrent*R=30 00*0.4= 12 00               
    TorqueID=limit(TorqueID,0,12000);
          
    int SetID= ScaleAnalog(int(HammerPos),SetHamMin_I,SetHamMax_I,MinTorque,MaxTorque); // Scale Speed Hammer position
    //Setpoint=0.2*(Setpoint-TorqueID);

    int E=SetID-TorqueID;
    Setpoint=limit(int(3*E+0.2*(E+prevE)+0.2*(E-prevE)),0,255);
    Setpoint=SensorFunctionOutput.RunningAverage(Setpoint,HIGH);
    prevE=SetID-TorqueID;    
}


int Controller::SpeedParameters(void){  
       switch (type_I) {
              case 0:   // 6...15Amp
                  MaxTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,1200,500); //scale min torque inputs to outputs maxtorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                  MinTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,450,0); //scale min torque inputs to outputs mintorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
              break;
              case 1:   // 6...15Amp
                  MaxTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,1100,570); //scale min torque inputs to outputs maxtorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                  MinTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,550,0); //scale min torque inputs to outputs mintorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
              break; 
              case 2:   // 1..6 Amp
                  MaxTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,1100,650); //scale min torque inputs to outputs maxtorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                  MinTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,600,0); //scale min torque inputs to outputs mintorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
              break;
              case 3:    // up to 1 Amp                      
                  MaxTorque= ScaleAnalog(MinTorqueSensor.RunningAverage(MinPin_I, LOW),0,1023,1100,700); //scale min torque inputs to outputs maxtorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
                  MinTorque= ScaleAnalog(MaxTorqueSensor.RunningAverage(MaxPin_I, LOW),0,1023,670,0); //scale min torque inputs to outputs mintorque limit->maxCurrent*R = 30A*0.4Ohm=12Volt
              break;
        }
}



int Controller::CalculateSpeed(int HammerPos,int current, int voltage){      
    SpeedID=voltage-current*R/10;
    SpeedID=limit(SpeedID,0,1200);
    int SetID= ScaleAnalog(int(HammerPos),SetHamMin_I,SetHamMax_I,1200-MaxTorque,1200-MinTorque); // Scale Speed Hammer position       
    int E=SetID-SpeedID;
    Setpoint=limit(int(0.5*E+0*(E+prevE)+0*(E-prevE)),0,255);
    
    Setpoint=SensorFunctionOutput.RunningAverage(Setpoint,HIGH);
    prevE=SetID-SpeedID; 
}


int Controller::Finalise(int HammerPos){
  
                        
    if(int(HammerPos)<=SetHamMin_I)
        Setpoint=0;
    
    if(digitalRead(ChokePin_I)==1 && HammerPos>=(SetHamMax_I)) //if Hammer Position is high and chocke ON then use relay, otherwise skip
        Setpoint=256; 
        
  return Setpoint;
  
  }
        


      
SetOutputs::SetOutputs(void){
  LastSetPoint=0;
  int SpeedBreak=0;
  }



 int SetOutputs::TransistorControl (int Setpoint, char pin) {

    //Setpoint is 0...255 (8bit PWM), BreakVal and TurboVal are 0...255 as well
    
    if (Setpoint<=255)       
      {
        if(Setpoint>0)         
              analogWrite(pin, Setpoint);// set inputs    
        else
            analogWrite(pin, 0);
     }
     
       return 0;
  }



 int SetOutputs::TurboControl(int Setpoint, char pin) {
    //Setpoint is 0...255 (8bit PWM), BreakVal and TurboVal are 0...255 as well
   
    if (Setpoint >= 256)
        digitalWrite(pin,HIGH);
      else
        digitalWrite(pin,LOW);
    // on simulator i get decreasing exponential signal. No idea why
    
       return 0;
  }


// that if setpoint does not change from 0 after strtup than do not use break, if setpoint is set to 0 for 10 seconds then break is off

int SetOutputs::CommitBreak(int Setpoint, char pin, char brValpin) {

  SpeedBreak= ScaleAnalog(analogRead(brValpin),0,1023,0,255);  //scale break to percents
  
  if ( Setpoint==0 &&  LastSetPoint!=0)
      breaksetpoint= millis();
     
  if (Setpoint <= 0 && millis()>(breaksetpoint+255-SpeedBreak) && millis()<(breaksetpoint+10000)) 
      analogWrite(pin, SpeedBreak); 
  else
      analogWrite(pin, 0); 

Serial.print("BreakFunc");
  LastSetPoint=Setpoint;
  return SpeedBreak;
  }


int SetOutputs::CommitBreakClassic(int Setpoint, char pin, char brValpin) {
  int SpeedBreak=0;
  SpeedBreak= ScaleAnalog(analogRead(brValpin),0,1023,0,255);  //scale break to percents
  

 //  if 
  
  if (Setpoint <= 0 ) 
      analogWrite(pin, SpeedBreak); 
  else
       analogWrite(pin, 0); 
    return 0;
  }















Measurements::Measurements(int RAintvalue){
		RACount=RAintvalue;
		NextRAValue=0;
		RAValue = 512;
}

int Measurements::RunningAverage(int PINnumber, bool data)
{
  int RawValue;
  if (data)
    RawValue = PINnumber;
  else
    RawValue = analogRead(PINnumber);
  
 /* Serial.print("RawValue: ");
  Serial.print(RawValue);*/
  
  RABuffer[NextRAValue++] = RawValue;  
  if (NextRAValue >= RACount)
    NextRAValue = 0; 
  for(int i=0; i< RACount; ++i)
    RAValue += RABuffer[i];
  RAValue /= (RACount+1);   
/*for(int i=0; i< RACount; ++i)
    {Serial.print(RABuffer[i]);
    Serial.print("-");
    }*/
  return (RAValue);
}










int ScaleAnalog(int X, int Xmin, int Xmax, int Ymin, int Ymax) {
      // analog input of arduino is 0 to 1023
      // then 0->Ymin, 1023->Ymax
      //integer is -32768...32768
      float y, k, b;
      int Yout;
      k=float(Ymax-Ymin)/float(Xmax-Xmin);
      b=float(Ymin)-k*float(Xmin);
      y=k*float(X)+b;
      Yout=int(y);
      return  Yout;
  }



int ScaleSquare(int X, int Xmin, int Xmax, int Ymin, int Ymax) {
      // analog input of arduino is 0 to 1023
      // then 0->Ymin, 1023->Ymax
      //integer is -32768...32768
      // y=a*pow((x-x0),0.5)+y0     
      //if we use lower power (ex 1/6) then we get bigger stage in the beginning)
      
      float a, y;
      int Yout; 
      a=(Ymax-Ymin)/sqrt(Xmax-Xmin);
      y=a*sqrt(float(X-Xmin))+Ymin;
      Yout=int(y);
      return  Yout;
  }



  
int limit (int value, int min, int max) {
  //limit min and max values of input
  if (value>=max)
    return max;
  if (value<=min)
    return min;

    return value;
  }

bool sign(int value) {
  // Sign math function
 return bool((value>0)-(value<0));
}
