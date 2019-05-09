#include <Arduino.h>
#include <Encoder.h>

#define COUNT_LOW 0
#define COUNT_HIGH 1023
#define ENC_MAX 2000
#define PULLY_R 1.0 //仮の数字
#define OFFSET_FORECE 200

//#define testAna  A0//テスト用アナログ出力
//#define testDig 21
#define Kp 2

//バーチャルな壁の位置，
//基準から位置が3000パルス(エンコーダ上で)動いた箇所に壁がある想定
#define WALLPOS 3000

int mode = 0; //0:力が釣り合っているモード, 1:力が発生するモード
float currentX = 0.0f; //x軸の座標
float prevX = 0.0f;

int motor1 = 14;  //モータ1
int enc1a = 27;   //モータ1エンコーダa相
int enc1b = 26;   //モータ1エンコーダb相

int motor2 = 4;  //モータ2
int enc2a = 35;   //モータ2エンコーダa相
int enc2b = 39;   //モータ2エンコーダb相

int motor3 = 33;  //モータ3
int enc3a = 18;   //モータ3エンコーダa相
int enc3b = 19;   //モータ3エンコーダb相

int motor4 = 25;  //モータ4
int enc4a = 34;   //モータ4エンコーダa相
int enc4b = 36;   //モータ4エンコーダb相

/*
int motor1 = 4;   //モータ1回転量
int enc1a = 35;   //モータ1エンコーダa相
int enc1b = 39;   //モータ1エンコーダb相

int motor2 = 25;  //モータ2
int enc2a = 34;   //モータ2エンコーダa相
int enc2b = 36;   //モータ2エンコーダb相

int motor3 = 14;  //モータ3
int enc3a = 27;   //モータ3エンコーダa相
int enc3b = 26;   //モータ3エンコーダb相

int motor4 = 33;  //モータ4
int enc4a = 18;   //モータ4エンコーダa相
int enc4b = 19;   //モータ4エンコーダb相
*/

int PWM_HZ = 40000;

Encoder myEnc1(enc1a,enc1b);
Encoder myEnc2(enc2a,enc2b);
Encoder myEnc3(enc3a,enc3b);
Encoder myEnc4(enc4a,enc4b);

float posX = 0.0f;
float posY = 0.0f;

/* 関数のプロトタイプ宣言 */
void motorOut(int, int);
void calcPos();
int SelectOutChannel(int);
int readPos(int);
int ForceFeedback(int, int);

void setup() {
  pinMode(motor1,OUTPUT);
  pinMode(motor2,OUTPUT);
  pinMode(motor3,OUTPUT);
  pinMode(motor4,OUTPUT);

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

  ledcSetup(1, PWM_HZ,10);
  ledcSetup(2, PWM_HZ,10);
  ledcSetup(3, PWM_HZ,10);
  ledcSetup(4, PWM_HZ,10);
  ledcAttachPin(motor1,1);
  ledcAttachPin(motor2,2);
  ledcAttachPin(motor3,3);
  ledcAttachPin(motor4,4);

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
  motorOut(1, OFFSET_FORECE);
  motorOut(2, OFFSET_FORECE);
  motorOut(3, OFFSET_FORECE);
  motorOut(4, OFFSET_FORECE);

  calcPos();
  
}


long counta = 0;

void calcPos(){

  int l1 = myEnc1.read();
  int l2 = myEnc2.read();
  int l3 = myEnc3.read();
  int l4 = myEnc4.read();


  l1 += 8000;
  l2 += 8000;
  l3 += 8000;
  l4 += 8000;

  //Serial.println(l1);


  posX = (l2*l2 - l3*l3)/4000000;
  posY = (l1*l1 - l2*l2)/4000000;


counta++;

if(counta%10000 == 0){
  
  
  Serial.print("(");
  Serial.print(posX);
  Serial.print(",");
  Serial.print(posY);
  Serial.println(")");
  
}

}


int readPos(int axis){
  int pos = 0;
  switch (axis)
  {
    // x座標
    case 0:
      pos = myEnc1.read();
      return pos;
      break;
  
    default:
      return 0;
      break;
  }
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
    case 1:
      return 1; //pwmの出力チャンネル
      break;
    case 2:
      return 2;
      break;
    case 3:
      return 3;
      break;
    case 4:
      return 4;
      break;
  
    default:
      return 0;
      break;
  }
}