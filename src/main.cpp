#include <Arduino.h>
#include <Encoder.h>

#define COUNT_LOW 0
#define COUNT_HIGH 1023
#define ENC_MAX 2000
#define PULLY_R 1.0 //仮の数字
#define OFFSET_FORECE 80
#define TAP 100 //平均化フィルタのタップ数

//#define testAna  A0//テスト用アナログ出力
//#define testDig 21
#define Kp 1

int mode = 0; //0:力が釣り合っているモード, 1:力が発生するモード
float currentX = 0.0f; //x軸の座標
float prevX = 0.0f;

int motor0 = 32;  //モータ0 14->32
int enc0a = 27;   //モータ0エンコーダa相
int enc0b = 14;   //モータ0エンコーダb相 26->14

int motor1 = 4;  //モータ1
int enc1a = 35;   //モータ1エンコーダa相
int enc1b = 39;   //モータ1エンコーダb相

int motor2 = 33;  //モータ2
int enc2a = 18;   //モータ2エンコーダa相
int enc2b = 19;   //モータ2エンコーダb相

int motor3 = 2;  //モータ3 32->2
int enc3a = 34;   //モータ3エンコーダa相
int enc3b = 36;   //モータ3エンコーダb相

int debugA = 25; //デバッグ用ピンA
int debugB = 26; //デバッグ用ピンB

//モータの仮想的な座標
int motorPos[4][2] = {{-1,-1}, {1,-1}, {1,1}, {-1,1}};

int PWM_HZ = 40000;

Encoder myEnc0(enc0a,enc0b);
Encoder myEnc1(enc1a,enc1b);
Encoder myEnc2(enc2a,enc2b);
Encoder myEnc3(enc3a,enc3b);

float posX = 0.0f;
float posY = 0.0f;
short shortFx = 0;
short shortFy = 0;
short prevFx = 0;
short prevFy = 0;

/* 関数のプロトタイプ宣言 */
void motorOut(int, int);
void motorsReset(void);
void calcPos();
void forceFeedback(float, float);
void calcMotorForce(int, int, float, float);
void sendSerial();
void receiveSerial();
float aveFilter(int, short);

float debugFu = 0.0f;
float debugFv = 0.0f;
float debugTheta = 0.0f;
float debugPhi = 0.0f;
float debugDeg = 0.0f;
float debugnx = 0.0f;
float debugny = 0.0f;

//平均化フィルタ
short force_prev[TAP][2]={};

void setup() {
  pinMode(motor0,OUTPUT);
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);

  ledcSetup(0, PWM_HZ,10);
  ledcSetup(1, PWM_HZ,10);
  ledcSetup(2, PWM_HZ,10);
  ledcSetup(3, PWM_HZ,10);
  ledcAttachPin(motor0,0);
  ledcAttachPin(motor1,1);
  ledcAttachPin(motor2,2);
  ledcAttachPin(motor3,3);

  //pinMode(testDig,OUTPUT);
  Serial.begin(115200);
}

void loop() {

  calcPos();
  //motorsReset();
  sendSerial();
  receiveSerial();
  //motorsReset();
  //short fx = aveFilter(0, shortFx);
  //short fy = aveFilter(1, shortFy);
  //forceFeedback(fx,fy);
  forceFeedback(shortFx,shortFy);
}

void motorsReset(){
  for(int i=0; i<4; i++){
    motorOut(i, OFFSET_FORECE);
  }
}

void calcPos(){

  int l0 = myEnc0.read();
  int l1 = myEnc1.read();
  int l2 = myEnc2.read();
  int l3 = myEnc3.read();


  l0 -= 8000;
  l1 -= 8000;
  l2 -= 8000;
  l3 -= 8000;

  //Serial.println(l1);
  posY = (l1*l1 - l2*l2)/400000;
  posX = (l0*l0 - l1*l1)/400000;
}

/*
  力ベクトルに応じて動かすモータを選択する関数
*/
void forceFeedback(float fx, float fy){
  float rad = atan2f(fy, fx); //x軸と力ベクトルのなす角を計算

  float deg = 180*rad/PI;
  debugDeg =  deg;

  //Serial.println(deg);
  //motorsReset();

  if(-45.0f < deg && deg < 45.0f){
    calcMotorForce(2, 1, fx, fy);
    motorOut(0,OFFSET_FORECE);
    motorOut(3,OFFSET_FORECE);
  }
  
  else if(45.0f < deg && deg <= 135.0f){
    calcMotorForce(3, 2, fx, fy);
    motorOut(0,OFFSET_FORECE);
    motorOut(1,OFFSET_FORECE);
  }
  else if((135.0f < deg && deg <= 180.0f) ||
    (-180.0f < rad && rad <= -135.0f)){
    calcMotorForce(0, 3, fx, fy);
    motorOut(1,OFFSET_FORECE);
    motorOut(2,OFFSET_FORECE);
  }
  else if(-135.0f < deg && deg <= -45.0f){
    calcMotorForce(1, 0, fx, fy);
    motorOut(2,OFFSET_FORECE);
    motorOut(3,OFFSET_FORECE);
  }
  else{
    calcMotorForce(1,0,0,0);
    //Serial.println("Error");
  }
}

/*
 モータに加わる力を計算する関数
*/
void calcMotorForce(int m1, int m2, float fx, float fy){
  float theta = (atan2f(motorPos[m1][1]-posY, motorPos[m1][0]-posX));
  float phi = (atan2f(motorPos[m2][1]-posY, motorPos[m2][0]-posX));

  float A = cosf(theta), B = cosf(phi);
  float C = sinf(theta), D = sinf(phi);

  float Fu = (D*fx - B*fy)/(A*D-C*B);
  float Fv = (A*fy - C*fx)/(A*D-C*B);

  debugFu = Fu;
  debugFv = Fv;

  debugTheta = theta*180/PI;
  debugPhi = phi*180/PI;
  motorOut(m1, (int)fabs(Kp*Fu));
  motorOut(m2, (int)fabs(Kp*Fv));
}

/*
モータを動かす関数 
motor:動かすモータ番号，value:値
*/
void motorOut(int motor, int value){
  if(fabs(value) < OFFSET_FORECE)
    value = OFFSET_FORECE;
  ledcWrite(motor,constrain(fabs(value),COUNT_LOW,COUNT_HIGH));

  if(motor == 0)
    dacWrite(debugA,(int)fabs(value/4));
}

float aveFilter(int axis, short currentForce){
  for(int i=0; i<TAP-1; i++)
    force_prev[i][axis] = force_prev[i+1][axis];
  force_prev[TAP-1][axis] = currentForce;

  short sum = 0;
  for(int i=0; i<TAP; i++)
    sum += force_prev[i][axis];

  return sum/TAP;
}

/*
シリアル通信でUnityに座標を送る関数
*/

void sendSerial(){
  Serial.println((String)posX+","+(String)posY);//1回で送れるようにする
  /*
  Serial.print(posX);
  Serial.print(",");
  Serial.print(posY);
  Serial.print(",");
  Serial.print(debugFu);
  Serial.print(",");
  Serial.println(debugFv);
  /*
  Serial.print(",");
  Serial.print(debugTheta);
  Serial.print(",");
  Serial.print(debugPhi);
  Serial.print(",");
  Serial.println(debugDeg);
  */
}

void receiveSerial(){
  if(Serial.available()){
    byte header = Serial.read();
    if(header == 0xFE){
      prevFx = shortFx;
      prevFy = shortFy;

      shortFx = Serial.read()*256;
      shortFx = shortFx+Serial.read();
      shortFy = Serial.read()*256;
      shortFy = shortFy+Serial.read();
    }
    else if(header == 0xFF){
      char ch = Serial.read();
      switch (ch)
      {
      case '0':
        motorPos[0][0] = posX;
        motorPos[0][1] = posY;
        break;

      case '1':
        motorPos[1][0] = posX;
        motorPos[1][1] = posY;
        break;
      case '2':
        motorPos[2][0] = posX;
        motorPos[2][1] = posY;
        break;
      case '3':
        motorPos[3][0] = posX;
        motorPos[3][1] = posY;
        break;
      
      default:
        break;
      }
    }
  }
}
