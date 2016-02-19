#define M1 10//PWM Pin for Motor1
#define M2 11//PWM Pin for Motor2
#define M1_A A5
#define M1_B A4
#define M2_A A3
#define M2_B A2
#define LED 12
#define V_5 13
#define base_speed 180
#define t 2 //delay time
#include <MsTimer2.h>
float kp=1.8, kd=0.2, ki=0.1, M1_speed=base_speed, M2_speed=base_speed, last_error=0;//universal constants
int left_sensor=0b000,mid_sensor=0b00,right_sensor=0b000;
void setup() 
{
  int i=2;
  for(i=2;i<10;i++)
    {
       pinMode(i,INPUT);//Reading pin 2-9 as input
    }
   //Serial.begin(9600);//for serial I/O of sensors. Comment out after debugging
   //pin declaration
   pinMode(LED,OUTPUT);//Using Arduino LED for checkpoint alert
   pinMode(V_5,OUTPUT);
   pinMode(M1,OUTPUT);
   pinMode(M2,OUTPUT);
   pinMode(M1_A,OUTPUT);
   pinMode(M2_A,OUTPUT);
   pinMode(M1_B,OUTPUT);
   pinMode(M2_B,OUTPUT);

   //pin initialisation
   digitalWrite(LED,LOW);//set LED as OFF
   digitalWrite(V_5,HIGH);
   digitalWrite(M1_A,HIGH);
   digitalWrite(M1_B,LOW);
   digitalWrite(M2_A,LOW);
   digitalWrite(M2_B,HIGH);
}


void loop() 
{
  read_sensor();//read the sensor data and get the error
  
  //For displaying error using serial I/O
  //delay(1500);
 // Serial.println(String("Error=")+ error);

  int error=calc_error();
  
  float pid=calc_PID(error);//Calculate PID value
  run_motors(pid);//Set the motor speed using calculated PID
}

int read_sensor(void)
{
  //
  //initialisation
  int arr[10],i,count=0;
  for(i=2;i<10;i++)
    {
      arr[i]=!digitalRead(i);//reading each sensor
      count+=arr[i];//Counting the number of sensors high
    }
   if(count>5)//Treating 5+ sensor high as checkpoint reached. Using MSTimer to autoswitch off LED after 1000ms.
   {
      MsTimer2::set(1500,checkpoint); // 1000ms period
      MsTimer2::start();//Start the timer
      digitalWrite(LED,HIGH);
   }
  //reading left sensor
  left_sensor=(arr[9])*2 + (arr[8]);
  mid_sensor=(arr[7])*8+(arr[6])*4 + arr[5]*2+arr[4];
  right_sensor=(arr[3])*2 + arr[2];  

  digitalWrite(M1_A,HIGH);
  digitalWrite(M1_B,LOW);
  digitalWrite(M2_A,LOW);
  digitalWrite(M2_B,HIGH);
}

int calc_error()
{
  if (left_sensor!=0 && right_sensor!=0)
    return 0;
  else if(left_sensor!=0)
    return 375;
 
  else if(right_sensor!=0)
    return -375;
  
  else if(mid_sensor!=0)
  {
    switch(mid_sensor)
    {
      case 0b0000:
        return 0;
      case 0b0001:
        return -100;
      case 0b0010:
        return -50;
      case 0b0100:
        return 50;
      case 0b1000:
        return 100;
      case 0b0011:
        return -80;
      case 0b0110:
        return 0;
      case 0b1100:
        return 80;
      case 0b0111:
        return -50;
      case 0b1110:
        return 50;
      case 0b1111:
        return 0;
      default :
        return 0;
    }
    
  }
  else
    return 0;
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
  //Serial.println(String("PID = ")+pid);
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
    motor2=255;
  
  if(motor2<0)//For correcting MIN PWM
    motor2=0;
  //Serial.println(String("Motor1=")+motor1+String(", Motor2=")+motor2); // For debugging. Comment out during actual operation.
  //delay(500);
  if(motor1>=250)
    M1_full();
  else if(motor2>=250)
    M2_full();
  else
  {
    analogWrite(M1,(int)motor1);
    analogWrite(M2,(int)motor2);
    delay(t/2);
  }
  /*delay(500);
  Serial.println(String("PID=")+ PID);
  Serial.println(String("Motor1=")+ motor1);
  Serial.println(String("char Motor1=")+ (int)motor1);
  Serial.println(String("Motor2=")+ motor2);
  Serial.println(String("char Motor2=")+ (int)motor2);*/
  
}
void checkpoint()//To be called when a checkpoint is reached.
{
  digitalWrite(LED,LOW);
  MsTimer2::stop();
}
void M2_full()
{
   digitalWrite(M2_A,HIGH);
   digitalWrite(M2_B,LOW); 
   analogWrite(M1,255);
   analogWrite(M2,255);
   delay(t);
}
void M1_full()
{
   digitalWrite(M1_A,LOW);
   digitalWrite(M1_B,HIGH);
   analogWrite(M1,255);
   analogWrite(M2,255);
   delay(t);
}

