#define M1 10//PWM Pin for Motor1
#define M2 11//PWM Pin for Motor2
#define base_speed 150
#include <MsTimer2.h>
float kp=2.5,kd=0.0,ki=0,M1_speed=base_speed,M2_speed=base_speed,last_error=0;//universal constants
void setup() 
{
  int i=2;
  for(i=2;i<10;i++)
    {
       pinMode(i,INPUT);//Reading pin 2-9 as input
    }
   Serial.begin(9600);//for serial I/O of sensors. Comment out after debugging
   pinMode(13,OUTPUT);//Using Arduino LED for checkpoint alert
   pinMode(12,OUTPUT);
   pinMode(M1,OUTPUT);
   pinMode(M2,OUTPUT);
   digitalWrite(13,HIGH);//set LED as OFF
   
}


void loop() 
{
  int error=read_sensor();//read the sensor data and get the error
  //For displaying error using serial I/O
  //delay(1500);
 // Serial.println(String("Error=")+ error);
  float pid=calc_PID(error);//Calculate PID value
  run_motors(pid);//Set the motor speed using calculated PID
}

int read_sensor(void)
{
  int arr[10],i,error=0,count=0;
  for(i=2;i<10;i++)
    {
      arr[i]=!digitalRead(i);//reading each sensor
      Serial.print(arr[i]);
      count+=arr[i];//Counting the number of sensors high
    }
   
   if(count>5)//Treating 5+ sensor high as checkpoint reached. Using MSTimer to autoswitch off LED after 1000ms.
   {
      MsTimer2::set(1500,checkpoint); // 1000ms period
      MsTimer2::start();//Start the timer
      digitalWrite(13,HIGH);
   }
   
   error=arr[5]+2*arr[4]+4*arr[3]+8*arr[2]-arr[6]-2*arr[7]-4*arr[8]-8*arr[9];//calculate error
   Serial.println(error);
   delay(500);
   return error;
}
float calc_PID(int ERR)
{
  float error=float(ERR);
  float pid=kp*error+kd*(error-last_error)+ki*(error+last_error);//PID algorithm
  /*Experimental block to increase speed at straight paths. Comment out if it proves to be unstable*/
  /*if(last_error==0 && error==0)
  {
    M1_speed+=2;
    M2_speed+=2;
  }
  else
  {
    M1_speed=base_speed;
    M2_speed=base_speed;
  }*/
  last_error=error;
  return pid;
}
void run_motors(float PID)
{
  float motor1=M1_speed+PID;//speed of Motor1
  float motor2=M2_speed-PID;//Speed of Motor2
  
  if(motor1>255)//For correcting MAX PWM
    motor1=255;
    
  if(motor1<0)//For correcting MIN PWM
    motor1=0;
  
  if(motor2>255)//For correcting MAX PWM
    motor1=255;
  
  if(motor2<0)//For correcting MIN PWM
    motor1=0;
  //Serial.println(String("Motor1=")+motor1+String(", Motor2=")+motor2); // For debugging. Comment out during actual operation.
  
  analogWrite(M1,(int)motor1);//Set MOTOR1 PWM
  analogWrite(M2,(int)motor2);//Set MOTOR2 PWM
  /*delay(500);
  Serial.println(String("PID=")+ PID);
  Serial.println(String("Motor1=")+ motor1);
  Serial.println(String("char Motor1=")+ (int)motor1);
  Serial.println(String("Motor2=")+ motor2);
  Serial.println(String("char Motor2=")+ (int)motor2);*/
  
}
void checkpoint()//To be called when a checkpoint is reached.
{
  digitalWrite(13,LOW);
  MsTimer2::stop();
}

