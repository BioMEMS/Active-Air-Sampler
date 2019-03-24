
/*
 This code controls the sampling flow rate and sampling volume/time of the Active Air Sampler.  A PID loop
 sets the value of the PWM signal which is used for controlling the sampling motor.  The battery voltage is also
 monitored and will halt sampling if the voltage is too low.  See README.md or the end of the code for the license.
 
 Journal Citation:
 Alexander G. Fung, Mei S. Yamaguchi, Mitchell M. McCartney, Alexander A. Aksenov, Alberto Pasamontes, Cristina E. Davis,
"SPME-based mobile field device for active sampling of volatiles", Microchemical Journal, Vo 146, 2019,Pages 407-413
https://doi.org/10.1016/j.microc.2019.01.012.
 */

 
const float flow_Setting=0.5;  //Desired Flow rate in LPM
const float Seconds2Minutes=1.0/60.0; //Converts Seconds to minutes
////////////////////////////////////////////////////////


const float sampling_Time=300; //Sampling Time in Seconds
float desired_Volume=sampling_Time/60.0*flow_Setting;
//Comment two lines above and uncomment 2 lines below if specific volume desired
//float desired_Volume= ;
//const float sampling_Time=desired_Volume/(flow_Setting)*60; //Sampling Time in Seconds


int pwm = 0;  //PWM signal from 0 to 255 to control 
const int pwm_Pin = 9;  //PWM output pin number
const int enable_Pin = 4; //Motor Driver Enable pin
const int battery_Pin = A5; //Battery input pin
const int sensor_Pin = A3; //Sensor Input pin
const int LED_PIN=12; //Red Warning LED Pin
boolean display_Last=true;  //Used to Display last sensor values before stopping the motor

float sensor_Bit; //Sensor Input Bit
float sensor_Flow=0;  //Sensor Flow rate in L/min
const float battery_Conversion = (43.2*5.0)/(1024.0*10.0); //Converts Battery input to Battery Voltage
float battery_Sense;  //Battery Voltage bit
float battery_Voltage;  //Battery Input Voltage

float prevIntegration_1=0; //Sum of previous integration terms for First PI controller
float error=0; //Error Term for PI controllers
float prev_Error=0;
boolean controller1 =true; //True until desired flow rate reached

//PID constants
const float kp_1=40.0; //P
const float ki_1=0.8;  //I
const float kd_1=1000.0; //D

float total_Volume=0;
float previous_Sensor_Flow=0;


float fpwm; //Floating point PWM, used to allow rounding for PWM
int battery_Counter=0;

unsigned long current_Time;  //Current Time in ms
unsigned long start_Time;    //Starting Time in ms
unsigned long previous_Time;  //Previous Sampling Time
unsigned long long startup_Time = 5000; //Lag time until until checking battery in ms
const unsigned long diff_Time=10;  //Difference between current_Time and previouis_Time in ms
void setup()
{
  //Initialize Communication with the Serial Monitor
  Serial.begin(38400);
  //Set PWM Frequency
  TCCR1B = TCCR1B & 0b11111000 | 0x01; //0x01=31372Hz PWM frequency
  //Set Up PWM Pin
  pinMode(pwm_Pin, OUTPUT);
  //Set up LED Pin
  pinMode(LED_PIN,OUTPUT);
  //Set up Enable Pin
  pinMode(enable_Pin, OUTPUT);
  //Initially have circuit turned off
  digitalWrite(enable_Pin, LOW);
  digitalWrite(LED_PIN, LOW);
 //Delay 2 seconds to set up Hand Vac
 delay(2000); 
  Serial.println("Active_Air_Sampler");
  Serial.print("Sampling Flow Rate: ");
  Serial.println(flow_Setting);
  Serial.print("Sampling time in seconds: ");
  Serial.println(sampling_Time);
 //Enable Hand Vac to be controlled
 digitalWrite(enable_Pin, HIGH);
 //But don't start it yet
 analogWrite(pwm_Pin,0);
 //Set the Start time
 start_Time=millis();
 //Which is also the current time
 current_Time=start_Time; 
}

void loop()
{
  //Previous time was old current_Time
  previous_Time=current_Time;
  //Assign new current time
  while (current_Time-previous_Time<diff_Time*1000)
  {
    current_Time=micros();
  }
  //If the Hand Vac hasn't been running for desired time
  if (total_Volume <desired_Volume)
    { 
      digitalWrite(LED_PIN, LOW);
      readValues();                //Read new Sensor values
      convertReadings();           //Convert to Proper Units
      flowIntegration();
      PID();  //Run PID controller
      displayValues();  //Print Values to Serial Port
    }
    else
    {
      if(display_Last)
      {
        readValues();         //Read new Sensor values
        convertReadings();    //Convert to proper units
        flowIntegration();
        displayValues();      //Display Values
        display_Last=false;   //Stop display values
      }
      stopMotor();            //Stop the motor
    }
    checkBattery();
    delay(1);                //Delay to give ADC time
}


void checkBattery()
{
if (current_Time/1000-start_Time>startup_Time)
  {
    if (battery_Voltage <17.5)
    {
       digitalWrite(LED_PIN, HIGH);
       if (battery_Voltage <16.5)
       {
         battery_Counter++;
         if (battery_Counter>4)
         {
           stopMotor();
           total_Volume=desired_Volume+1000;
           delay(200);
           digitalWrite(LED_PIN, LOW);
           delay(200);
         }
       } 
    }
  }
}
void PID()
{
    prev_Error=error;
    error=flow_Setting-sensor_Flow;
    prevIntegration_1 = prevIntegration_1+(error+prev_Error)/2.0*float(diff_Time);
    fpwm=kp_1*error+ki_1*(prevIntegration_1)+kd_1*(error-prev_Error)/float(diff_Time);
    
    pwm = constrain(floor(fpwm+0.5),0,255);       //Constrain it to what can reasonably be attained
    //digitalWrite(enable_Pin, HIGH);
    analogWrite(pwm_Pin,pwm);
}

void flowIntegration()
{
  //Integrate using trapezoidal rule, time is in milliseconds
  total_Volume+= (sensor_Flow+previous_Sensor_Flow)*(diff_Time)/120000.0; //12000=0.5*60000 
}


void stopMotor()
{
    pwm = 0; //
    analogWrite(pwm_Pin, pwm);
    digitalWrite(enable_Pin, LOW); //Turn off Gate Driver
}

void readValues()
{
  battery_Sense=0;
  sensor_Bit=0;
  //Oversample for average values
  for (int i=1;i<=16;i++)
  {
    //Read Battery and Sensor Values
    battery_Sense += analogRead(battery_Pin);
    sensor_Bit += analogRead(sensor_Pin);
  }
  battery_Sense/=16;
  sensor_Bit/=16;
}

void convertReadings()
{
  //Save Previous Flow
  previous_Sensor_Flow=sensor_Flow;
  //Convert to Sensor Flow Rate  
  sensor_Flow = 2.786*sensor_Bit*sensor_Bit/1000000.0-9.652*sensor_Bit/10000.0+0.1166;//Sensor 2;
  //Convert Battery Voltage bit to actual voltage
  battery_Voltage = battery_Sense * battery_Conversion;
}

void displayValues()
//Displays Desired Values
{
  //Running Time
  Serial.print(current_Time/1000-start_Time);
  Serial.print(",");
  Serial.print(sensor_Bit);
  Serial.print(",");
  Serial.print(sensor_Flow,3);
  Serial.print(",");
  Serial.print(pwm);
  Serial.print(",");
  Serial.print(total_Volume,3);
  Serial.print(",");
  Serial.print(battery_Voltage);
  Serial.println("");
  
}
/*Active Air Sampler is the proprietary property of The Regents of the University of California (“The Regents.”)
Copyright © 2019 The Regents of the University of California, Davis campus. All Rights Reserved.  
Redistribution and use in source and binary forms, with or without modification, are permitted by nonprofit, research institutions for research use only, provided that the following conditions are met:
•  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
• Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
• The name of The Regents may not be used to endorse or promote products derived from this software without specific prior written permission. 
The end-user understands that the program was developed for research purposes and is advised not to rely exclusively on the program for any reason.
THE SOFTWARE PROVIDED IS ON AN "AS IS" BASIS, AND THE REGENTS HAVE NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS. THE REGENTS SPECIFICALLY DISCLAIM ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES, INCLUDING BUT NOT LIMITED TO  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES, LOSS OF USE, DATA OR PROFITS, OR BUSINESS INTERRUPTION, HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY WHETHER IN CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
If you do not agree to these terms, do not download or use the software.  This license may be modified only in a writing signed by authorized signatory of both parties.
For commercial license information please contact copyright@ucdavis.edu.
*/
