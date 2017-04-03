//为了在kalman filter里用上声呐，这个版本里改变了行动模式，分两步到达goalpoint/3.15
//增加了x，y，theta三坐标的卡尔曼滤波/3.20
//硬件上增加了一个声呐。安装在右侧。对应函数sonar1()/3.21
//通过函数拟合，修正了电机对罗盘的磁干扰/3.23

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
int count_per_meter=78;    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!




Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float goalx=95.0,goaly=109.0;
int T,V,R,i,j;
float D;
int vel;        ////left 22 per turn   right 25 per turn
int encoder_pinL = 2; 
int encoder_pinR = 3; // The pin the encoder is connected 
int speedrpm; // desired speed 
int desired_angle;   
unsigned long timeold=0;
bool left_error=false;
float sum_sonar_reading;
   ///////////////////////////////////////////


  ////////////////////////////////////kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk///////////////////////
  float kd=0.5;  ///P gain
  float kd1=1.0;  ///  hmc5883 angle control gain 
  float kd2=0.1;// D gain 
  ////////////////////////////kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk ///////////////////////
  
unsigned int rpmL,rpmR;     // rpm reading
volatile short pulsesL,pulsesR,current_count;  // number of pulses
unsigned long count_total;
unsigned long timeL,timeR,dTL,dTR; 
int motorpwmL,motorpwmR;
//// move control///
int turn_flag,move_distance,move_count,move_count_x,move_count_y;
 bool firsttime;
 int  current_speedL=0;
 int  current_speedR=0;
 bool speed_saved=false;
 bool localized=false;
 int  last_errorL,last_errorR;
 float tan_errorL=0.0;
 float tan_errorR=0.0;
 bool shut_down_controller=false;
 float  Dis_1,Dis_2;

 //////////kalman filter var@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 float KF_current_x=0.0,KF_current_y=0.0, KF_current_theta=0.0;
 float varianceC=0.697,varianceM=0.065,varianceMc=0.001;//based on measurement, C=control M=sonar Mc=compass
 float KF_kx,KF_ky,KF_ktheta;
 float px=1.0,py=1.0,ptheta=1.0;
 unsigned long  KF_countL=0, KF_countR=0;
 int m_count;
 float d_theta;
 float KF_last_reading;
 bool x=true,y=false;

 void counterL()  ///interrupt0
 {
    //Update count
      pulsesL++;
      count_total++; 
      KF_countL++;   
 }
  void counterR()   ///interrupt1
 {
    //Update count
      pulsesR++;
      KF_countR++;    
 }

 float Gaussian(float mu, float sigma, float x) 
    { 
    return exp(-pow(mu-x,2)/pow(sigma,2)/2.0)/sqrt(2.0*PI*pow(sigma,2));
    }
 
/////////////////////////////////////////////////////////////P-D controller/////////////////////////////////////
void controllerL(int speedrpm)
{ 
  int des_speed=speedrpm;
  int errorL=des_speed-rpmL;
  //if (abs(last_errorL-errorL)>abs(last_errorL)) errorL=last_errorL;//////filter
  motorpwmL+=kd*errorL;
  
  int predict_errorL=2*errorL-last_errorL;
  motorpwmL+=kd2*predict_errorL;          
  last_errorL=errorL;
  if(motorpwmL<50) motorpwmL=50;
  if(motorpwmL>255) motorpwmL=255;
  
  }
  void controllerR(int speedrpm)
{
  int des_speed=speedrpm;  //22
  int errorR=des_speed-rpmR;
    // if (abs(last_errorR-errorR)>abs(last_errorR)) errorR=last_errorR;//////filter
  motorpwmR+=kd*errorR;
  int predict_errorR=2*errorR-last_errorR;
  motorpwmR+=kd2*predict_errorR;
  last_errorR=errorR;
   if(motorpwmR<50) motorpwmR=50;
  if(motorpwmR>255) motorpwmR=255;
  }


void sgn(int a)
{ int b;
  if(a>0) b=1;
  if (a==0) b=0;
  if(a<0) b=-1;
  return b;
  }
  /////////////sonar,detect distance in CM.
float sonar ()
{ 
digitalWrite(12,LOW);
///////  delay(1);
digitalWrite(12,HIGH);
delayMicroseconds(15);
digitalWrite(12,LOW);
 float Dis=pulseIn(13,HIGH)/58.0;
return (Dis);
}
float sonar1 ()
{ 
digitalWrite(10,LOW);
///////  delay(1);
digitalWrite(10,HIGH);
delayMicroseconds(15);
digitalWrite(10,LOW);
 float Dis=pulseIn(11,HIGH)/58.0;
return (Dis);
}

void forward()
  {
  
    digitalWrite(4,LOW);
    digitalWrite(5,HIGH);
    digitalWrite(7,LOW);
    digitalWrite(8,HIGH);
    analogWrite(6,motorpwmL);  //vl
    analogWrite(9,motorpwmR);  //vr
   
    }

void halt()
{
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  }


  /////////hmc5883
  float bearing()
  {
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.147;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI; 
  if(heading > 2*PI)
    heading -= 2*PI;
  float heading_actual=125.1*sin(0.008*heading)+0.27*sin(1.003*heading-0.534);//fitted curve 
  if(heading_actual < 0)
    heading_actual += 2*PI;
  return ( heading_actual);
    }

  void turn( int desired_bearing)
 {
 int D_angle=desired_bearing-bearing()*180/PI;
 if(D_angle>180)  D_angle-=360;
 if(D_angle<-180) D_angle+=360; // make D_angle between -pi to pi
     int minium_angle=8;
     int wheelc=60+abs(D_angle);
     if(wheelc>100) wheelc=100;
     if(wheelc<80)  wheelc=80;
     while(D_angle>5||D_angle<-5)
  {
     while(D_angle>5||D_angle<-5)
   {
      if(D_angle>0)
     {digitalWrite(4,LOW);
      digitalWrite(5,HIGH); //left wheel
      digitalWrite(7,HIGH);
      digitalWrite(8,LOW); //right wheel
      analogWrite(6,wheelc);  //vl
      analogWrite(9,wheelc);
      }
   
     if(D_angle<0) 
     {digitalWrite(4,HIGH);
      digitalWrite(5,LOW);
      digitalWrite(7,LOW);
      digitalWrite(8,HIGH);
      analogWrite(6,wheelc);  //vl
      analogWrite(9,wheelc); } //vr  // turn left}
      D_angle=desired_bearing-bearing()*180/PI;
      if(D_angle>180)  D_angle-=360;
      if(D_angle<-180) D_angle+=360;
      wheelc=60+abs(D_angle);
      if(wheelc>100) wheelc=100;
      if(wheelc<80)  wheelc=80;
    }
      //minium_angle=minium_angle-2;
      //if(minium_angle<3) minium_angle=3;
  
     halt();
     delay(500);  
     D_angle=desired_bearing-bearing()*180/PI;
     if(D_angle>180)  D_angle-=360;
     if(D_angle<-180) D_angle+=360;
     }
     halt();
  }

//////////////////////////////////////////kalman_filter\\\\\\\\\\\\\\\\\\\\\\\@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void kalman_f(bool dir)
{  
  if(dir){     /////in x dir
  float KFheading=bearing(); 
  float sonar_reading=(sonar()+5.0)*cos(KFheading);
  float sonar_reading1=(sonar1()+5.0)*cos(KFheading);
  if(KFheading<0.523||KFheading>5.757) 
    {sonar_reading=193.0-sonar_reading;sonar_reading1=153.0-sonar_reading1;}//fitted curve
   Serial.print("X sonar_reading= ");
   Serial.println(sonar_reading);
   Serial.print("X sonar_reading1= ");
   Serial.println(sonar_reading1);
   m_count=(KF_countL+KF_countR)/2;

  ///////////////////////////kalman x//////////////////////////////////
  KF_current_x=KF_current_x+100.0*m_count*cos(KFheading)/count_per_meter;///x1
  if(fabs(sonar_reading- KF_current_x)>30.0) {px=px+varianceC;} //if bad reading, do not make correction
   else 
   {
    px=px+varianceC;//////////x2
    KF_kx=px/(px+varianceM);//////x3
    KF_current_x=KF_current_x+KF_kx*(sonar_reading-KF_current_x);//////x4
    px=(1-KF_kx)*px;///////x5
    }
/////////////////////////kalman theta//////////////////////////
 KF_current_theta=KF_current_theta+((KF_countL-KF_countR)/count_per_meter)/0.11;///////////theta1
 ptheta=ptheta+varianceC;                       ///////////////////////////////////////////theta2
 KF_ktheta=ptheta/(ptheta+varianceMc);                 ////////////////////////////////////theta3
 KF_current_theta=KF_current_theta+KF_ktheta*(KFheading-KF_current_theta);           //////theta4
 ptheta=(1-KF_ktheta)*ptheta;                                               ///////////////theta5
         
/*  
  d_theta=((KF_countL-KF_countR)/count_per_meter)/0.22;///theta=(A-B)/D
 Dis_2+=(100.0*m_count/(2.0*count_per_meter))*sin(d_theta);*/

/////////////////////////kalman y/////////////////////////////
 KF_current_y=KF_current_y+(100.0*m_count/(2.0*count_per_meter))*sin(KF_current_theta);     ///y1
 if(fabs(sonar_reading1- KF_current_y)>30.0) {py=py+varianceC;} //if bad reading, do not make correction
 else {
 
 py=py+varianceC;                                                                    //////////y2
 KF_ky=py/(py+varianceM);                                                                //////y3
 KF_current_y=KF_current_y+KF_ky*(sonar_reading1-KF_current_y);                          //////y4
 py=(1-KF_ky)*py;                                                                       ///////y5
  }
  }

  
else{   //in y dir
  float KFheading=bearing();
  float sonar_reading=(sonar()+5.0)*sin(KFheading);
  float sonar_reading1=(sonar1()+5.0)*sin(KFheading);
 if(KFheading<2.093||KFheading>1.047) sonar_reading=153.0-sonar_reading;
 else sonar_reading1=193.0-sonar_reading1;
  Serial.print("Y sonar_reading= ");
  Serial.println(sonar_reading);
  Serial.print("Y sonar_reading1= ");
  Serial.println(sonar_reading1);
  m_count=(KF_countL+KF_countR)/2;

  ///////////////////kalman y //////////////////////////
  KF_current_y=KF_current_y+100.0*m_count*sin(KFheading)/count_per_meter;
 if(fabs(sonar_reading-KF_current_y)>30.0) {py=py+varianceC;}
  else{
 py=py+varianceC;
 KF_ky=py/(py+varianceM);
 KF_current_y=KF_current_y+KF_ky*(sonar_reading-KF_current_y);
 py=(1-KF_ky)*py;
 }
//////////////////////kalman theta////////////////////
 KF_current_theta=KF_current_theta+((KF_countL-KF_countR)/count_per_meter)/0.11;///////////theta1
 ptheta=ptheta+varianceC;                       ///////////////////////////////////////////theta2
 KF_ktheta=ptheta/(ptheta+varianceMc);                 ////////////////////////////////////theta3
 KF_current_theta=KF_current_theta+KF_ktheta*(KFheading-KF_current_theta);           //////theta4
 ptheta=(1-KF_ktheta)*ptheta;                                               ///////////////theta5
////////////////kalman x/////////////////////////////
/*Dis_1+=(100.0*m_count/(2.0*count_per_meter))*sin(d_theta);*/
 KF_current_x=KF_current_x+(100.0*m_count/(2.0*count_per_meter))*cos(KF_current_theta);     ///x1
 if(fabs(sonar_reading1- KF_current_x)>30.0) {px=px+varianceC;} //if bad reading, do not make correction
 else {
 
 px=px+varianceC;                                                                    //////////x2
 KF_kx=px/(px+varianceM);                                                                //////x3
 KF_current_x=KF_current_x+KF_kx*(sonar_reading1-KF_current_x);                          //////x4
 px=(1-KF_kx)*px;                                                                       ///////x5
  }

 
 }
 
  }


  
 void Particle_filter()/////////////////////////////////56565757653452432453//////////////////////////////////
  {
    //这里D=20cm为每次Robot向前移动的距离，目前没有办法得到,chujun, please help me!
//Particle会每次更新，更新到最后所有的particle的mean都十分接近于测量值，fabs(mean(D_x[])-PF_current_x)<k, k=1cm or 2cm，可调整；
//return particle的mean作为 x,y,theta;
//每一次robot向前移动一个D，都要delay（300）；
   
   
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   int m;
   float x_P[100]={0};
   float y_P[100]={0};
   float theta_P[100]={0};

   bool PF_flag =true;
   bool PF_flag1=true;
   bool PF_flag2=true;

   float xx=0;
   float yy=0;
   float ttheta=0;
   int n,ii,jj,k;
    Serial.println("c p1 ");
////////////////////////////////// Gaussian fuction defination////////////////////////////////////// 
   float Gauss[100]=
   {
    0.75,  -0.25,  0.86, -0.29,  0.85, 0.53, -0.18,  -0.72,  -0.87,  -0.23,  -0.37,  -0.34,  0.82, -0.25,  0.95, -0.44,  0.81, -0.44,  0.15, 0.81, -0.35,  -0.37,  1.67, 0.79, -0.36,  0.54, -0.3, 0.59, 1.18, -1.34,  0.86, 1.22, 0.04, 1.46, 0.13, -1.03,  -1.83,  -0.28,  0.6,  0.26, 0.35, -0.48,  0.12, -1.37,  -0.63,  -0.68,  0.43, -0.01,  -0.96,  -0.01,  -0.58,  -0.56,  0.72, 0.09, 0.33, 0.74, 0.15, 0.46, 0.57, 0.98, 0.4,  1.18, 0.02, -0.04,  1.42, -0.43,  0,  0.77, 0.13, 1.17, 0.86, 0.24, -0.65,  0.47, -1.15,  0.2,  0.67, 0.18, 0.73, 1.7,  0.77, 0.22, 0.54, 0.36, -1.1, -0.35,  1.02, -0.04,  0.49, -0.84,  0.05, 0.5,  -1.14,  0.29, -0.15,  -0.78,  -0.03,  -1.58,  -1.78,  -0.98
   };
   
    for ( m = 0; m < 100; m++)                // initialize particle
      {
        xx = random(0, 193);
        x_P[m]=xx;                            // particle x coords value
        yy = random(0, 153);
        y_P[m]=yy;                            // particle y coords value
        ttheta = random(0, 2*PI);
        theta_P[m]=ttheta;                    // particle theta coords value
      }

        Serial.print("x_P[]=");
        for(m=0;m<100;m++)
          {Serial.println(x_P[m]);}
        Serial.print("y_P[]=");
        for(m=0;m<100;m++)
          {Serial.println(y_P[m]);}
        Serial.print("theta_P[]=");
        for(m=0;m<100;m++)
          {Serial.println(y_P[m]);}
        
///////////////////////////// Particle filter variables /////////////////////////////////
   
   float PF_current_x=0.0,PF_current_y=0.0, PF_current_theta=0.0;
   float sumx_P=1000, sumy_P=1000, sumtheta_P=1000;
   D= (KF_countL+KF_countR)*PI*6.8/40; ///using counter and rpm to calculate, make D=20cm every time
   float D_theta=(KF_countL-KF_countR)/11;
   
/////////////////////////////////////// Particle filter in locolization ////////////////////////////////////
do{    
   if(!localized)                 
  {
  unsigned long old_time=millis();
  forward();
  controllerL(60);
  controllerR(60); 
  if (millis()-old_time>200)
    {
      halt();
      old_time=millis();
    
    
  // we should only put the robot heading to the lower right corner                               
  float PFheading=bearing();                  
  float sonar_reading=(sonar()+5.0)*cos(PFheading);
  float sonar_reading1=(sonar1()+5.0)*cos(PFheading);
  if(PFheading<0.523||PFheading>5.757) 
    {sonar_reading =193.0-sonar_reading;
     sonar_reading1=153.0-sonar_reading1;}//fitted curve
     
  /////////////////////// Particle filter parameters //////////////////////////////
  PF_current_x=sonar_reading;
  PF_current_y=sonar_reading1;
  PF_current_theta=PFheading;
  for (m = 0; m < 100; m++) 
    { 
    xx=x_P[m];  
    x_P[m]=xx+D*cos(PFheading)+Gauss[random(0,99)];   //update x info
  
    yy=y_P[m];  
    y_P[m]=yy+D*cos(PFheading)+Gauss[random(0,99)];  //update y info
    
    ttheta=theta_P[m];  
    theta_P[m]=ttheta+D_theta+Gauss[random(0,99)]/11; //update theta info
    }
    Serial.print("Gaussian x_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(x_P[m]);}
    Serial.print("Gaussian y_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(y_P[m]);}
    Serial.print("Gaussian theta_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(theta_P[m]);}
   
   float D_x[100]={0}, D_y[100]={0}, D_theta[100]={0};
   float w_x[100]={0}, w_y[100]={0}, w_theta[100]={0};
   float sum_wx=0, sum_wy=0, sum_wtheta=0; 
   for (m = 0; m < 100; m++)     //give each particle a weight depend on the distance to measured location
     {
     D_x[m] = fabs(PF_current_x - x_P[m]);
     D_y[m] = fabs(PF_current_x - y_P[m]);
     D_theta[m] = fabs(PF_current_theta - theta_P[m]);
     w_x[m] =  fabs(Gaussian(0,px+varianceC+varianceM,D_x[m]));
     sum_wx = sum_wx+w_x[m]; 
     w_y[m] =  fabs(Gaussian(0,py+varianceC+varianceM,D_y[m]));
     sum_wy = sum_wy+w_y[m]; 
     w_theta[m] = fabs(Gaussian(0,ptheta+varianceMc,D_theta[m]));
     sum_wtheta = sum_wtheta+w_theta[m]; 
     }
   for (m = 0; m < 100; m++)   //weights normalization 
     {
      w_x[m] = w_x[m]/sum_wx;
      w_y[m] = w_y[m]/sum_wy;
      w_theta[m] = w_theta[m]/sum_wtheta;
     }
   
////////////////////////////////////////resampling//////////////////////////////////////////////////////////
   
   //float sortw_x[100]=w_x[]; sortw_y[100]=w_y[]; sortw_theta[100]=w_theta[];
   float rw_x[100]={0}, rw_y[100]={0}, rw_theta[100]={0};
   
   for (ii = 0; ii < 100; ii++)     // sort weight of x from min to max
     {
      for (jj=0;jj<100-jj;jj++)
      {                                                        
      if(w_x[ii]> w_x[jj + 1]) 
       {                                                      
        float w_temp = w_x[ii]; 
        w_x[ii] = w_x[jj + 1]; 
        w_x[jj+1] = w_temp;  
        float PF_temp = x_P[i]; 
        x_P[ii] = x_P[jj + 1]; 
        x_P[jj+1] = PF_temp; 
       }  
      }
     }                                                                                 
   
   for (ii = 0; ii < 100; ii++)    // sort weight of y from min to max
     {
      for (jj=0;jj<100-jj;jj++)
      {                                                        
      if(w_y[ii]> w_y[jj + 1]) 
       {                                                      
        float w_temp = w_y[ii]; 
        w_y[ii] = w_y[jj + 1]; 
        w_y[jj+1] = w_temp;  
        float PF_temp = y_P[ii]; 
        y_P[ii] = y_P[jj + 1]; 
        y_P[jj+1] = PF_temp; 
       }  
      }
     }                   
   
    for (ii = 0; ii < 100; ii++)    // sort weight of theta from min to max
      {
      for (jj=0;jj<100-jj;jj++)
       {                                                        
      if(w_theta[ii]> w_theta[jj + 1]) 
        {                                                      
        float w_temp = w_theta[ii]; 
        w_theta[ii] = w_theta[jj + 1]; 
        w_theta[jj+1] = w_temp;  
        float PF_temp = theta_P[ii]; 
        theta_P[ii] = theta_P[jj + 1]; 
        theta_P[jj+1] = PF_temp; 
        }  
       }
      }                   
   for(m=0;m<100;m++)            //cumulate all sorted weights
    {
      int n=m;
      do
      {rw_x[m]+=w_x[n];
       rw_y[m]+=w_y[n];
       rw_theta[m]+=w_theta[m];
       n=n-1;}
      while(n==0);
     }
   
     float timesx[100]={0};      // select particles by weight, the highest weight particle will be selected 
     float timesy[100]={0};      // more and then the 2nd highest one and then the 3nd one till we count to 100
     float timestheta[100]={0};                   
     int sum_timesx=0;
     int sum_timesy=0;
     int sum_timestheta=0;
     float updatex_P[100]={0};
     float updatey_P[100]={0};
     float updatetheta_P[100]={0};
     ////////////////////////////////////select updated particle x/////////////////////
     m=100;  
     do
     {
      timesx[m]=round(rw_x[m]/0.25);
      if(sum_timesx<100)
       {sum_timesx=sum_timesx+timesx[m];
       m=m-1;}
      }
      while(m==0);
      //Serial.print("timesx[]=");
      //Serial.println(timesx[]);
     n=100;
     k=sum_timesx-100;
     for(m=100;m>0;m=m-1)
      {
         do
         {
          updatex_P[n]=x_P[m];
          timesx[m]=timesx[m]-1;
          n=n-1;
          sum_timesx=sum_timesx-1;
         }
         while (timesx[m]==0&&sum_timesx>k);
      }
     ////////////////////////////////////select updated particle y/////////////////////
     m=100;  
     do
     {
      timesy[m]=round(rw_y[m]/0.25);
      if(sum_timesy<100)
       {sum_timesy=sum_timesy+timesy[m];
       m=m-1;}
      }
      while(m==0);
      //Serial.print("timesy[]=");
      //Serial.println(timesy[]);
     n=100;
     k=sum_timesy-100;
     for(m=100;m>0;m=m-1)
      {
        do
         {
          updatey_P[n]=y_P[m];
          timesy[m]=timesy[m]-1;
          n=n-1;
          sum_timesy=sum_timesy-1;
         }
         while(timesy[m]==0&&sum_timesy>k);
      }
     ////////////////////////////////////select updated particle theta/////////////////////
     m=100;  
     do
     {
      timestheta[m]=round(rw_theta[m]/0.25);
      if(sum_timestheta<100)
       {sum_timestheta=sum_timestheta+timestheta[m];
       m=m-1;}
      }
      while(m==0);
      //Serial.print("timestheta[]=");
      //Serial.println(timestheta[]);
     n=100;
     k=sum_timestheta-100;
     for(m=100;m>0;m=m-1)
      {
        do
         {
          updatetheta_P[n]=theta_P[m];
          timestheta[m]=timestheta[m]-1;
          n=n-1;
          sum_timestheta=sum_timestheta-1;
         }
         while(timestheta[m]==0&&sum_timestheta>k);
      }
     ///////////////print the updated particle////////////
      Serial.print("updatex_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(updatex_P[m]);}
      Serial.print("updatey_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(updatey_P[m]);}
      Serial.print("updatetheta_P[]=");
      for(m=0;m<100;m++)
          {Serial.println(updatetheta_P[m]);} 
    
    //////////////// estimate whether the mean of the particle is good enough//////////
    sumx_P=0; 
    sumy_P=0; 
    sumtheta_P=0;
    for(m=1;m<100;m++)
      {x_P[m]=updatex_P[m];
       sumx_P=sumx_P+x_P[m];
       y_P[m]=updatey_P[m];
       sumy_P=sumy_P+y_P[m];
       theta_P[m]=updatetheta_P[m];
       sumtheta_P=sumtheta_P+theta_P[m];}
   }
  }
      delay(300);
}
  while( fabs((sumx_P/100)-PF_current_x)<3&&fabs((sumx_P/100)-PF_current_x)<3&&fabs((sumtheta_P/100)-PF_current_theta)<0.1);
     localized = true;
     
     Dis_1= sumx_P/100;                       // this is the final PF parameter to KF in the next
     Dis_2= sumy_P/100;
     KF_current_theta=sumtheta_P/100;
     
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
void setup() {    /////////////////////////////////////////////////////////////////setup///////////////////////////////////////////////////////////////////////////////
  pinMode(4,OUTPUT);//motor Left
  pinMode(5,OUTPUT);//motor Left
  pinMode(6,OUTPUT);//motor Left vel
  pinMode(7,OUTPUT);// motor Right
  pinMode(8,OUTPUT);// motor Right
  pinMode(9,OUTPUT);// motor Right vel
  pinMode(10,OUTPUT);
  pinMode(11,INPUT);
  pinMode(12,OUTPUT);//sonar triger
  pinMode(13,INPUT);//sonar reciver
  
  pinMode(encoder_pinL, INPUT);
  pinMode(encoder_pinR, INPUT);
Serial.begin(9600);
   //Use statusPin to flash along with interrupts
   //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on FALLING (change from HIGH to LOW)
   attachInterrupt(0, counterL, FALLING);
   attachInterrupt(1, counterR, FALLING);
   // Initialize
   pulsesL = 0;
   pulsesR = 0;
   rpmL = 0;
   rpmR = 0;
   motorpwmL=100;
   motorpwmR=100;
   turn_flag=0;
   move_distance=0;
   move_count=0;
   move_count_x=0;
   move_count_y=0;
   current_count=0;
   count_total=0;
   dTL=0;
   dTR=0;
   timeL=0;
   timeR=0;
   last_errorL=0;
   last_errorR=0;
   firsttime=true;

   
   
  
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }


}
/////////////////////////////////////main /////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  Serial.println("csd p2 ");
if(!localized) Particle_filter();

/*
 /////////////////////////////////////////////////////////////////////////////////////////////
   ///////first, localize robot                                                           ///
  if(!localized)                                                                          ///
  { localized=true;                                                                       ///
   turn(180); //turn north                                                                ///
   delay(100);                                                                            ///
   //get 10 sonar readings                                                                ///
   float ap=0.0;                                                                          ///
   float a_buff[10];                                                                      ///
   int j=0;                                                                               ///
   for(int i=0;i<10;i++)                                                                  ///
   {                                                                                      ///
     a_buff[i]=sonar();                                                                   ///
    Serial.print("a_buff[i]=");
    Serial.println(a_buff[i]);
    delay(200);                                                                           ///          
    }                                                                                     ///
                                                                                          ///
    for(i = 0; i < 9 ; i++) {                                                             ///
      if(a_buff[0]> a_buff[i + 1]) {                                                      ///
        float a_temp = a_buff[0];                                                         ///
        a_buff[0] = a_buff[i + 1];                                                        ///
        a_buff[i + 1] = a_temp;                                                           ///
      }                                                                                   ///
    } 
                                                                                          ///
    float a_fliter=1.2*a_buff[0];                                                         ///
    for(i = 1; i < 10 ; i++)                                                              ///
    {                                                                                     ///
if(a_buff[i]<a_fliter) {ap+=a_buff[i];j++;}                                               ///
    }                                                                                     /// 
                                                                                          ///
    Dis_1=ap/j+5;                                                                         ///
     Serial.print("j=");
     Serial.println(j);   
     Serial.print("Dis_1=");
     Serial.println(Dis_1);   
  //  Serial.print("sum1= ");                                                             ///
 // Serial.println(sum_sonar_reading);                                                    ///
 // Serial.print("j1= ");                                                                 ///
  //Serial.println(j);                                                                    ///
    delay(100);                                                                           ///
    //////////////////////////////                                                        ///
    turn(270); ///turn east                                                               ///
    delay(100);                                                                           ///
    //get 10 sonar readings                                                               ///
    ap=0.0;                                                                               ///
     j=0;                                                                                 ///
    for(int i=0;i<10;i++)                                                                 ///
   {                                                                                      ///
     a_buff[i]=sonar(); 
        Serial.print("a_buff[i]=");
        Serial.println(a_buff[i]);                                                        ///
        delay(200);                                                                       ///          
    }                                                                                     ///
                                                                                          ///
    for(i = 0; i < 9 ; i++) {                                                             ///
      if(a_buff[0]> a_buff[i + 1]) {                                                      ///
        float a_temp = a_buff[0];                                                         /// 
        a_buff[0] = a_buff[i + 1];                                                        ///
        a_buff[i + 1] = a_temp;                                                           ///
      }                                                                                   ///
    }                                                                                     ///
     a_fliter=1.2*a_buff[0];                                                              ///
    for(i = 1; i < 10 ; i++)                                                              ///
    {                                                                                     ///
if(a_buff[i]<a_fliter) {ap+=a_buff[i];j++;}                                               ///
    }                                                                                     ///
    Dis_2=ap/j+5; 
    Serial.print("j2=");
    Serial.println(j);   
    Serial.print("Dis_2=");
    Serial.println(Dis_2);                                                                ///
                                                                                          ///
  }                                                                                       ///
/////////////////////////////////////////////////////////////////////////////////////////////
*/



// After localize, calculate travel_dis and heading
for(int g=0;g<6;g++)
{
px=1.0; py=1.0;ptheta=1.0; /////reset p
float dx=goalx-Dis_1;
float dy=goaly-Dis_2; 
move_count_x=abs(dx*count_per_meter/100);
move_count_y=abs(dy*count_per_meter/100);//////////////////////////////95 ///tune every time//////////////////////70count 1 meter//////////

/// turn to x direction

if(dx>0){turn(0);desired_angle=0;}
if(dx<0) {turn(180);desired_angle=180;} ///should add calman filter for angle theta here later,,,,
KF_current_x=Dis_1;
KF_current_y=Dis_2;
KF_current_theta=bearing();
delay(500);
/// move in x 
timeL=millis();timeR=millis();  
count_total=0;
 KF_countL=0;
 KF_countR=0;
while(count_total<move_count_x)
{
 
int angle_error=desired_angle-bearing()*180/PI;

if(angle_error<-180) {angle_error=angle_error+360;left_error=true;}
if(left_error&&(angle_error>5)) 
{ 
  angle_error-=10;
  shut_down_controller=true;
  if(!speed_saved){
  current_speedL=motorpwmL;
  current_speedR=motorpwmR;
  speed_saved=true;
  }
  motorpwmL=current_speedL+kd1*angle_error;
  motorpwmR=current_speedR-kd1*angle_error;
  if(motorpwmL<50) motorpwmL=50;
  if(motorpwmL>255) motorpwmL=255;
  if(motorpwmR<50) motorpwmL=50;
  if(motorpwmR>255) motorpwmL=255;
  } 
if((!left_error)&&(angle_error>5))
{ 
  angle_error+=10;
  shut_down_controller=true;
  if(!speed_saved){
  current_speedL=motorpwmL;
  current_speedR=motorpwmR;
  
  speed_saved=true;
  }
  motorpwmL=current_speedL+kd1*angle_error;
  motorpwmR=current_speedR-kd1*angle_error;
  if(motorpwmL<50) motorpwmL=50;
  if(motorpwmL>255) motorpwmL=255;
  if(motorpwmR<50) motorpwmL=50;
  if(motorpwmR>255) motorpwmL=255;
  }

if(angle_error<5) shut_down_controller=false;////end of hmc5883 control

 /////////////////////kalman filter////////////////
if(millis()-timeold>=200)
{
  timeold=millis();
  kalman_f(x);
 Serial.print("KF_current_x= ");
 Serial.println(KF_current_x);
 Serial.print("KF_current_y= ");
 Serial.println(KF_current_y);
 Serial.print("m_count= ");
 Serial.println(m_count);
  KF_countL=0;
  KF_countR=0;
  }


forward();
delayMicroseconds(20);
if(!shut_down_controller){
if(pulsesL==5){pulsesL=0;dTL=millis()-timeL;timeL=millis();rpmL=13043/dTL; controllerL(80);}
if(pulsesR==5){pulsesR=0;dTR=millis()-timeR;timeR=millis();rpmR=13043/dTR; controllerR(80);}
}
else{pulsesL=0;pulsesR=0;}
}
      
  halt();
  delay(200);////////////////make sure last0.2s is count in
  kalman_f(x);
 Serial.print("KF_current_x= ");
 Serial.println(KF_current_x);
 Serial.print("KF_current_y= ");
 Serial.println(KF_current_y);
 Serial.print("m_count= ");
 Serial.println(m_count);
  KF_countL=0;
  KF_countR=0;     
  delay(1100);


//turn to y direction
if(dy>0) {turn(90);desired_angle=90;}
if(dy<0) {turn(270);desired_angle=270; }       // again, need calman filter
KF_current_theta=bearing();
delay(500);

timeL=millis();timeR=millis(); 
count_total=0;
  KF_countL=0;
  KF_countR=0;
while(count_total<move_count_y)
{
  forward();
int angle_error=desired_angle-bearing()*180/PI;
if(angle_error>180) angle_error=angle_error-360;
if(angle_error<-180) angle_error=angle_error+360;

if(angle_error>3) { 
  angle_error-=3;
  shut_down_controller=true;
  if(!speed_saved){
  current_speedL=motorpwmL;
  current_speedR=motorpwmR;
  speed_saved=true;
  }
  motorpwmL=current_speedL+kd1*angle_error;
  motorpwmR=current_speedR-kd1*angle_error;
  if(motorpwmL<50) motorpwmL=50;
  if(motorpwmL>255) motorpwmL=255;
  if(motorpwmR<50) motorpwmL=50;
  if(motorpwmR>255) motorpwmL=255;
  }
  
if(angle_error<-3) { 
  angle_error+=3;
  shut_down_controller=true;
  if(!speed_saved){
  current_speedL=motorpwmL;
  current_speedR=motorpwmR;
  
  speed_saved=true;
  }
  motorpwmL=current_speedL+kd1*angle_error;
  motorpwmR=current_speedR-kd1*angle_error;
  if(motorpwmL<50) motorpwmL=50;
  if(motorpwmL>255) motorpwmL=255;
  if(motorpwmR<50) motorpwmL=50;
  if(motorpwmR>255) motorpwmL=255;
  }

if(abs(angle_error)<3) 
{shut_down_controller=false;
} 

/////////////////////kalman filter////////////////
if(millis()-timeold>=200)
{
  timeold=millis();
  kalman_f(y);
  Serial.print("KF_current_x= ");
  Serial.println(KF_current_x);
  Serial.print("KF_current_y= ");
  Serial.println(KF_current_y);
  Serial.print("m_count= ");
  Serial.println(m_count);
  KF_countL=0;
  KF_countR=0;
  }

delayMicroseconds(20);

if(!shut_down_controller){
if(pulsesL==5){pulsesL=0;dTL=millis()-timeL;timeL=millis();rpmL=13043/dTL; controllerL(80);}
if(pulsesR==5){pulsesR=0;dTR=millis()-timeR;timeR=millis();rpmR=13043/dTR; controllerR(80);}
}
else{pulsesL=0;pulsesR=0;}
}

 delay(200);
 kalman_f(y);
 Serial.print("KF_current_x= ");
 Serial.println(KF_current_x);
 Serial.print("KF_current_y= ");
 Serial.println(KF_current_y);
 Serial.print("m_count= ");
 Serial.println(m_count);
  KF_countL=0;
  KF_countR=0;
  Dis_1=KF_current_x;
  Dis_2=KF_current_y;
  halt();
  delay(500);

float Dx=fabs(goalx-Dis_1);
float Dy=fabs(goaly-Dis_2);
if(Dx<20.0&&Dy<20.0) while(1){halt();}
}
while(1){halt();}
}
               
                
  

 
