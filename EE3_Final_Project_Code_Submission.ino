#include <ECE3.h>
#include <cmath>
uint16_t sensorValues[8]; // right -> left, 0 -> 7
float fsenseVals[8];
const float offsetweight[8] = {-8,-4,-2,-1,1,2,4.5,8};
const float minVal[8] = {820, 770, 680, 620, 620, 620, 690, 670};
const float maxVal[8] = {1750, 1780, 1500, 1270, 1300, 1650, 1680,1800};
int leftSpd;
int rightSpd;
int basePWM = 70;
float Kp = 0.03; 
float Kd = 3.2; 
float errorChange;
float lastVal;
int somespeed = 0;
int donutCount = 0;
const int left_nslp_pin=31; 
const int left_dir_pin=29; 
const int left_pwm_pin=40; 
const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

///////////////////////////////////
void setup() {
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH); //let left wheel on

 pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW); //high voltage or low voltage
  digitalWrite(right_nslp_pin,HIGH);
  
  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();
  // set the data rate in bits/second for serial data transmission
  //Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  errorChange = 0;
  lastVal = 0;
}

void loop() {
  leftSpd = 170;
  rightSpd = 170;
  basePWM = 45; 
  digitalWrite(right_dir_pin,LOW);
  float weighted = 0;
  ECE3_read_IR(sensorValues);
  int endingDonut = 0;
  for (unsigned char i = 0; i < 8; i++)
  {
    fsenseVals[i] = sensorValues[i]; //convert uint to float
    fsenseVals[i] -= minVal[i];
    fsenseVals[i]/= maxVal[i];
    fsenseVals[i] *= 1000.00;
    if (fsenseVals[i] >= 950){
      endingDonut ++;
    }
    weighted += fsenseVals[i]*offsetweight[i];
  }
  //obtained the weighted error value of sensors
  weighted/=4.0;
  errorChange = weighted - lastVal;
  somespeed = Kd*(errorChange);
  if(endingDonut >= 7){
    donutCount++;
    //if sensed the cross section twice, do a donut
    if(donutCount == 2){
      digitalWrite(right_dir_pin,HIGH);
      analogWrite(left_pwm_pin,leftSpd); //0-255
      analogWrite(right_pwm_pin,rightSpd);
      delay(300);
    }
    else if (donutCount > 2){ //if sensed the cross section again, then stop
      digitalWrite(left_nslp_pin,LOW);
      digitalWrite(right_nslp_pin,LOW);
    }
  }else{
    //code for the four different scenarios described in the flow diagram 
    if (weighted > 0){
      if (errorChange > 0){
        rightSpd = basePWM+somespeed; 
        basePWM -= Kp*weighted;
        leftSpd = basePWM;
        
      }else{
         basePWM += Kp*weighted;
         rightSpd = basePWM;
         leftSpd = basePWM+somespeed;
      }
    }
    else {
      if (errorChange < 0){
        leftSpd = basePWM-somespeed; 
        basePWM += Kp*weighted;
        rightSpd = basePWM;
      }else{
        basePWM -= Kp*weighted;
        leftSpd = basePWM;;
        rightSpd = basePWM-somespeed;
      }
    }
  }
  //update the previous error value to obtain error change
  lastVal = weighted;
  analogWrite(left_pwm_pin,leftSpd); //0-255
  analogWrite(right_pwm_pin,rightSpd);
  digitalWrite(LED_RF, HIGH);
  digitalWrite(LED_RF, LOW);
  }
