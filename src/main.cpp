#include <Arduino.h>
#include <Encoder.h>

#define COUNT_LOW 0
#define COUNT_HIGH 1023
#define ENC_MAX 2000
#define PULLY_R 1.0 //仮の数字
#define OFFSET_FORECE 100

//#define testAna  A0//テスト用アナログ出力
//#define testDig 21
#define Kp 100

int mode = 0; //0:力が釣り合っているモード, 1:力が発生するモード
float currentX = 0.0f; //x軸の座標
float prevX = 0.0f;

int motor0 = 14;  //モータ0
int enc0a = 27;   //モータ0エンコーダa相
int enc0b = 26;   //モータ0エンコーダb相

int motor1 = 4;  //モータ1
int enc1a = 35;   //モータ1エンコーダa相
int enc1b = 39;   //モータ1エンコーダb相

int motor2 = 33;  //モータ2
int enc2a = 18;   //モータ2エンコーダa相
int enc2b = 19;   //モータ2エンコーダb相

int motor3 = 25;  //モータ3
int enc3a = 34;   //モータ3エンコーダa相
int enc3b = 36;   //モータ3エンコーダb相

//モータの仮想的な座標
int motorPos[4][2] = {{-1,-1}, {1,-1}, {1,1}, {-1,1}};

int PWM_HZ = 40000;

Encoder myEnc0(enc0a,enc0b);
Encoder myEnc1(enc1a,enc1b);
Encoder myEnc2(enc2a,enc2b);
Encoder myEnc3(enc3a,enc3b);

float posX = 0.0f;
float posY = 0.0f;

/* 関数のプロトタイプ宣言 */
void motorOut(int, int);
void calcPos();
void forceFeedback(float, float);
void calcMotorForce(int, int, float, float);
int SelectOutChannel(int);
int ForceFeedback(int, int);
void checkControlPoint();

void setup() {
  pinMode(motor0,OUTPUT);
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);

/*
  pinMode(enc1a,INPUT);
  pinMode(enc1b,INPUT);
  pinMode(enc2a,INPUT);
  pinMode(enc2b,INPUT);
  pinMode(enc3a,INPUT);
  pinMode(enc3b,INPUT);
  pinMode(enc4a,INPUT);
  pinMode(enc4b,INPUT);
*/

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

long oldPosition = 0;

void loop() {
  /*
  ledcWrite(0,200);
  ledcWrite(1,200);
  ledcWrite(2,200);
  ledcWrite(3,200);
  */

  /*
  motorOut(0, OFFSET_FORECE);
  motorOut(1, OFFSET_FORECE);
  motorOut(2, OFFSET_FORECE);
  motorOut(3, OFFSET_FORECE);
  */

  calcPos();
  //forceFeedback(posX,posY);
  checkControlPoint();
  
}


long counta = 0;

void calcPos(){

  int l0 = myEnc0.read();
  int l1 = myEnc1.read();
  int l2 = myEnc2.read();
  int l3 = myEnc3.read();


  l0 += 8000;
  l1 -= 8000;
  l2 -= 8000;
  l3 += 8000;

  //Serial.println(l1);

  posY = (l1*l1 - l2*l2)/4000000;
  posX = (l0*l0 - l1*l1)/4000000;


  counta++;


  if(counta%10000 == 0){
    
    Serial.print("(");
    Serial.print(posX);
    Serial.print(",");
    Serial.print(posY);
    Serial.println(")");
    
  }
}

void checkControlPoint(){
  float r = 5.0f;
  float square = posX*posX+posY*posY;

  //円の領域内に入ったら
  if(square < r*r){
    float fx = posX/square;
    float fy = posY/square;
    forceFeedback(Kp*fx,Kp*fy);
  }
  //領域外の時
  else{
    motorOut(0, OFFSET_FORECE);
    motorOut(1, OFFSET_FORECE);
    motorOut(2, OFFSET_FORECE);
    motorOut(3, OFFSET_FORECE);
  }
}

/*
  力ベクトルに応じて動かすモータを選択する関数
*/
void forceFeedback(float fx, float fy){
  float x0 = 1.0f, y0 = 0.0f;
  float rad = atan2f(fy-y0, fx-x0); //x軸と力ベクトルのなす角を計算

  float deg = 180*rad/PI;

  //Serial.println(deg);

  if(-45.0f < deg && deg < 45.0f){
    calcMotorForce(1, 2, fx, fy);
    //Serial.println("0");
  }
  else if(45.0f < deg && deg <= 135.0f){
    calcMotorForce(2,3, fx, fy);
    //Serial.println("1");
  }
  else if((135.0f < deg && deg <= 180.0f) ||
    (-180.0f < rad && rad <= -135.0f)){
    calcMotorForce(3,0, fx, fy);
    //Serial.println("2");
  }
  else if(-135.0f < deg && deg <= -45.0f){
    calcMotorForce(0,1, fx, fy);
   //Serial.println("3");
  }
  else{
    Serial.println("Error");
  }
}

/*
 モータに加わる力を計算する関数
*/
void calcMotorForce(int m1, int m2, float fx, float fy){
  float x0 = 1.0f, y0 = 0.0f;

  float theta = atan2f(motorPos[m1][1]-y0, motorPos[m1][0]-x0);
  float phi = atan2f(motorPos[m2][1]-y0, motorPos[m2][0]-x0);

  float A = cosf(theta), B = cosf(phi);
  float C = sinf(theta), D = sinf(phi);

  float Fv = (A*fy - C*fx)/(A*D-C*B);
  float Fu = (fx - Fv*B)/A;
  
  Serial.print(Fu);
  Serial.print(",");
  Serial.println(Fv);
  for(int i=0; i<4; i++)
    motorOut(i,OFFSET_FORECE);
  motorOut(m1, (int)fabs(Fu));
  motorOut(m2, (int)fabs(Fv));
}

int ForceFeedback(int current,int target){
  int errPos = target - current;
  int ctrlValue = errPos*Kp;
  return ctrlValue;
}

/*
モータを動かす関数 
motor:動かすモータ番号，value:値
*/
void motorOut(int motor, int value){
  ledcWrite(motor,constrain(fabs(value),COUNT_LOW,COUNT_HIGH));
}

/*
出力に使うpwmの出力チャンネルを選ぶ関数
 */
int SelectOutChannel(int motor){
  switch (motor)
  {
    case 0:
      return 0; //pwmの出力チャンネル
      break;
    case 1:
      return 1;
      break;
    case 2:
      return 2;
      break;
    case 3:
      return 3;
      break;
  
    default:
      return 0;
      break;
  }
}