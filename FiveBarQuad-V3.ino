// This program uses my Inverse Kinematics function to move the FiveBarQuad robot - 20/07/2019
#include <Servo.h>
const int nbSrv=8;                                          // Number of servos used
Servo Srv[nbSrv];                                           // Servos table
int SrvOn[nbSrv]={1,1,1,1,1,1,1,1};                         // Switch servos table 1=On 0=Off

//Rectangle movement 
int FRx[]= { 16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16,    -16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16};
int BRx[]= {-16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16,     16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16};
int FLx[]= { 16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16     -16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16};
int BLx[]= {-16,-14,-12,-10, -8, -6, -4, -2,  0,  2,  4,  6,  8, 10, 12, 14, 16,     16, 14, 12, 10,  8,  6,  4,  2,  0, -2, -4, -6, -8,-10,-12,-14,-16};

int FRy[]= {  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,     10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
int BRy[]= { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,      5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};
int FLy[]= { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,      5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};
int BLy[]= {  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,     10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};

int lgTab = sizeof(FRx)/sizeof(int);

const char DOUT_TRIGGER =11;  const char DIN_ECHO =12;  float distance = 1000;        //US sensor HC-SR04 connectors and variables

void  setup() {
  Serial.begin(9600);
  pinMode(DOUT_TRIGGER, OUTPUT);                              // Ultrasonic sensor HC-SR04 initialization
  digitalWrite(DOUT_TRIGGER, LOW);                            // Ultrasonic sensor HC-SR04 initialization
  pinMode(DIN_ECHO, INPUT);                                   // Ultrasonic sensor HC-SR04 initialization
  
  pinMode(0,INPUT_PULLUP);                                    // start button attachment
  
  Srv[0].attach(3);  Srv[1].attach(4);                        // Front Right leg
  Srv[2].attach(7);  Srv[3].attach(8);                        // Back Right leg
  Srv[4].attach(5);  Srv[5].attach(6);                        // Front left leg
  Srv[6].attach(9);  Srv[7].attach(10);                       // Back left leg
    
  Srv[0].write(90);  Srv[1].write(90);                       // set servo to neutral point
  Srv[2].write(90);  Srv[3].write(90);                       // set servo to neutral point
  Srv[4].write(90);  Srv[5].write(90);                       // set servo to neutral point
  Srv[6].write(90);  Srv[7].write(90);                       // set servo to neutral point
  
  while( digitalRead(0) );                                    // wait for start button pressed
}

void  loop(){
  US_detect();
  if (distance <= 40){for(int i=0;i<8;i++) TurnRight();} else  Forward();
}

void Forward(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the target points from the beginning to the end
    InverseKinematics(FRx[i],FRy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[i],BRy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(FLx[i],FLy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(BLx[i],BLy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  }  
}

void Backward(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the target points from the end to the beginning
    InverseKinematics(FRx[lgTab-1-i],FRy[lgTab-1-i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[lgTab-1-i],BRy[lgTab-1-i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(FLx[lgTab-1-i],FLy[lgTab-1-i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(BLx[lgTab-1-i],BLy[lgTab-1-i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  } 
}

void TurnLeft(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the right legs to turn left
    InverseKinematics(FRx[i],FRy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[i],BRy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(BRx[i],BRy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(FRx[i],FRy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  }  
}

void TurnRight(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the left legs to turn right
    InverseKinematics(FLx[i],FLy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BLx[i],BLy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(BLx[i],BLy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(FLx[i],FLy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);    
  } 
}

void InverseKinematics(int Px, int Py, Servo srvL, Servo srvR, int srvLon, int srvRon){       // below, 12 lines to calculate the angle of the servos
  int g=34;                                                     //  1- Interval between the two servos
  float A1x=-17, A1y=128, A2x=A1x+g, A2y=A1y;                   //  2- Values of the servos positions
  float a1=96, c1=32, a2=a1, c2=c1;                             //  3- Values of the length of the leg's bars
  float d1=abs(A1y-Py), d2=abs(A2y-Py);                         //  4- values of the d sides of the virtual right-angle triangles
  float e1=abs(A1x-Px), e2=abs(A2x-Px);                         //  5- value of e sides of the virtual right-angle triangles
  float b1=sqrt(sq(d1)+sq(e1)), b2=sqrt(sq(e2)+sq(d2));         //  6- value of b sides (hypotenuse) of the virtual right-angle triangles 
  if ( b1>(a1+c1) || b2>(a1+c1) ){                              //    test if the target point is reachable
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!");
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!");
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!\n");
    return;
  }
  float A_1=(acos((sq(b1)+sq(c1)-sq(a1))/(2*b1*c1)))*57.296;    //  7- Value of the A angle of the left triangle with cosine law. Radian to degree convertion
  float A_2=(acos((sq(b2)+sq(c2)-sq(a2))/(2*b2*c2)))*57.296;    //  8- Value of the A angle of the right triangle with cosine law. Radian to degree convertion
  float D1=(acos((sq(g)+sq(b1)-sq(b2))/(2*b1*g)))*57.296;       //  9- Value of the D left angle of the center triangle with cosine law. Radian to degree convertion
  float D2=(acos((sq(g)+sq(b2)-sq(b1))/(2*b2*g)))*57.296;       // 10- Value of the D right angle of the center triangle with cosine law. Radian to degree convertion

  int S1=round(180-A_1-D1);                                     // 11- Angle value of the left servo
  int S2=round(A_2+D2);                                         // 12- Angle value of the right servo
  if ( S1<5 || S2>175 ){                                        // if target angle is too short or too large, it is reject
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!");
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!");
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!\n");
    return;
  }
  if (srvLon) srvL.write(S1);                                   // set target Left servo position if servo switch is On
  if (srvRon) srvR.write(S2);                                   // set target Right servo position if servo switch is On
  delay(2);
}

void US_detect(){                                               // US sensor HC-SR04 function sensor
  digitalWrite(DOUT_TRIGGER, LOW);  delayMicroseconds(2);
  digitalWrite(DOUT_TRIGGER, HIGH); delayMicroseconds(10);
  digitalWrite(DOUT_TRIGGER, LOW);
  distance= pulseIn(DIN_ECHO, HIGH) / 58.0;
//  Serial.print("\tDistance ");Serial.print(distance);Serial.print(" cm\n ");                
}

/*
//Rectangle - alternative
int FRx[]= { 15, 12,  9,  6,  3,  0, -3, -6, -9,-12,-15,    -15,-12, -9, -6, -3,  0,  3,  6,  9, 12, 15};
int BRx[]= {-15,-12, -9, -6, -3,  0,  3,  6,  9, 12, 15,     15, 12,  9,  6,  3,  0, -3, -6, -9,-12,-15};
int FLx[]= { 15, 12,  9,  6,  3,  0, -3, -6, -9,-12,-15,    -15,-12, -9, -6, -3,  0,  3,  6,  9, 12, 15};
int BLx[]= {-15,-12, -9, -6, -3,  0,  3,  6,  9, 12, 15,     15, 12,  9,  6,  3,  0, -3, -6, -9,-12,-15};

int FRy[]= { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,     15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
int BRy[]= { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,     10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
int FLy[]= { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,     10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
int BLy[]= { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,     15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15};
 */
