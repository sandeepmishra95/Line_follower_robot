#define M1 10//PWM Pin for Motor1
#define M2 11//PWM Pin for Motor2
#define M1_A A5
#define M1_B A4
#define M2_A A3
#define M2_B A2
#define LED 12
#define V_5 13
#define base_speed 150
#include <MsTimer2.h>
float kp=1, kd=0.0, ki=0, M1_speed=base_speed, M2_speed=base_speed, last_error=0;//universal constants
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
  int error=read_sensor();//read the sensor data and get the error
  //For displaying error using serial I/O
  //delay(1500);
 // Serial.println(String("Error=")+ error);
  float pid=calc_PID(error);//Calculate PID value
  run_motors(pid);//Set the motor speed using calculated PID
}

int read_sensor(void)
{
  /*analogWrite(M1,0);//Set MOTOR1 PWM
  analogWrite(M2,0);//Set MOTOR2 PWM
  delay(50);*/
  int arr[10],i,data=0b00000000,count=0,error;
  for(i=2;i<10;i++)
    {
      arr[i]=!digitalRead(i);//reading each sensor
      count+=arr[i];//Counting the number of sensors high
      data+=arr[i]<<(i-2);
    }
   if(count>5)//Treating 5+ sensor high as checkpoint reached. Using MSTimer to autoswitch off LED after 1000ms.
   {
      MsTimer2::set(1500,checkpoint); // 1000ms period
      MsTimer2::start();//Start the timer
      digitalWrite(LED,HIGH);
   }
   
   error=calc_error(data,count);
   return error;
}

int calc_error(int data,int count)
{
  switch(count)
  {
    case 1:
       //for 1 sensor on a line
          if(data==0b10000000)
            return -(base_speed+255);
          else if(data==0b01000000)
            return -(base_speed+210);
          else if(data==0b00100000)
            return -(base_speed+170);
          else if(data==0b00010000)
            return -(base_speed+base_speed);
          else if(data==0b00001000)
            return 0;
          else if(data==0b00000100)
            return (170-base_speed);
          else if(data==0b00000010)
            return (210-base_speed);
          else if(data==0b00000001)
            return (255-base_speed);
    case 2:
        //for 2 sensor on a line
        if(data==0b11000000)
          return -(base_speed+255);
        else if(data==0b01100000)
          return -(base_speed+200);
        else if(data==0b00110000)
          return -(base_speed+170);
        else if(data==0b00011000)
          return 0;
        else if(data==0b00001100)
          return (170-base_speed);
        else if(data==0b00000110)
          return (200-base_speed);
        else
          return (255-base_speed);
    case 3:
        //for 3 on a line
        if(data==0b11100000)
          return -(base_speed+255);
        else if(data==0b01110000)
          return -(base_speed+190);
        else if(data==0b00111000)
          return -(base_speed+170);
        else if(data==0b00011100)
          return (170-base_speed);
        else if(data==0b00001110)
          return (190-base_speed);
        else 
          return (255-base_speed);
    case 4:
        if(data==0b11110000)
          return -(base_speed+255);
        else if(data==0b01111000)
          return -(base_speed+180);
        else if(data==0b00111100)
          return 0;
        else if(data==0b00011110)
          return (180-base_speed);
        else
          return (255-base_speed);
    case 5:
         if(data==0b11111000)
          return -(base_speed+255);
         else if (data==0b01111100)
          return -(base_speed+190);
         else if (data==0b00111110)
          return (190-base_speed);
         else
          return (255-base_speed);
   default:
          return 0;
  } 
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
    
  if(motor1<-255)//For correcting MIN PWM
    motor1=-255;
  
  if(motor2>255)//For correcting MAX PWM
    motor1=255;
  
  if(motor2<-255)//For correcting MIN PWM
    motor1=-255;
  //Serial.println(String("Motor1=")+motor1+String(", Motor2=")+motor2); // For debugging. Comment out during actual operation.
  //delay(500);
  runM1(motor1);//Set MOTOR1 PWM with Direction
  runM2(motor2);//Set MOTOR2 PWM with Direction
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
void runM1(float val)
{
  if (val>0)
  {
    digitalWrite(M1_A,HIGH);
    digitalWrite(M1_B,LOW);
  }
  else
  {
    val=-val;
    digitalWrite(M1_A,LOW);
    digitalWrite(M1_B,HIGH);
  }
  analogWrite(M1,int(val));
}

void runM2(float val)
{
  if (val>0)
  {
    digitalWrite(M2_A,LOW);
    digitalWrite(M2_B,HIGH);
  }
  else
  {
    val=-val;
    digitalWrite(M2_A,HIGH);
    digitalWrite(M2_B,LOW);
  }
  analogWrite(M2,int(val));
}
