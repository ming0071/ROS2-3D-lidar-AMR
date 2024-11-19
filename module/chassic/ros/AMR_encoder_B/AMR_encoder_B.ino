
#include "ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

#define outputA 2 //定義 outputA = 2
#define outputB 40 //定義 outputB = 7
int aState; //定義 aState 為 int 類型變數
int aLastState; //定義 aLastState 為 int 類型變數
int encoderValueB = 0;
long previousMillis = 0;
long currentMillis = 0;
long clearPreviousMillis;
double t = 0;
int s = 0;

std_msgs::Float64 encoder2;
ros::Publisher encoder2_value("/encoder2_value", &encoder2);
//初始化設定↓↓↓
void setup() 
{
  nh.initNode();
  nh.advertise(encoder2_value);
  pinMode (outputA,INPUT); //埠口模式設定：outputA 設為 輸入
  pinMode (outputB,INPUT); //埠口模式設定：outputB 設為 輸入
  attachInterrupt (digitalPinToInterrupt(2), test, CHANGE); //啟用中斷函式(中斷0,test函式,CHANGE模式)
  //Serial.begin (9600); //Serial通訊鮑率設為9600
  aLastState = digitalRead (outputA); //將初始outputA的讀取值 設給 aLastState
}

//主程式運作區↓↓↓
void loop() 
{
  clearPreviousMillis = micros();
  if (clearPreviousMillis - previousMillis > 100000)
  {
    encoderValueB = 0;
    encoder2.data = 0;
  }
  encoder2_value.publish(&encoder2);
  nh.spinOnce();
  delay(20);  
  //Serial.print("Velocity: "); //透過serial印出字串 Position:
  //Serial.println(encoderValueA); //透過serial印出 counter 值
}

//test函式
void test()
{
  aState = digitalRead(outputA); //將outputA的讀取值 設給 aState

  if (aState != aLastState)
  { //條件判斷，當aState 不等於 aLastState時發生

    if (digitalRead(outputB) != aState) 
    { //條件判斷，當outputB讀取值 不等於 aState時發生i
      encoderValueB ++; //計數器+1
    } 
    else 
    {
      encoderValueB --; //計數器-1
    }
    //Serial.print("Pulses: "); //透過serial印出字串 Position:
    //Serial.println(encoderValueA); //透過serial印出 counter 值
  }
  if(encoderValueB == 1 || encoderValueB == -1)
  {
    previousMillis = micros();
  }
  if(encoderValueB > 10 || encoderValueB < -10)
  {
    if(encoderValueB>0)
    {
      s=-1;
    }
    else if(encoderValueB<0)
    {
      s=1;
    }
    currentMillis = micros();
    t = currentMillis - previousMillis;
    encoder2.data = (float)(((s * 753982.23686) / t)/20);
    encoderValueB = 0;
    s = 0;
  }   
  aLastState = aState; //將aState 最後的值 設給 aLastState
}
