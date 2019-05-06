#include <Arduino.h>
#include <Encoder.h>

#define COUNT_LOW 0
#define COUNT_HIGH 1023
#define ENC_MAX 2000
#define PULLY_R 1.0 //仮の数字
#define OFFSET_FORECE 40

//#define testAna  A0//テスト用アナログ出力
//#define testDig 21
#define Kp 2

//バーチャルな壁の位置，
//基準から位置が3000パルス(エンコーダ上で)動いた箇所に壁がある想定
#define WALLPOS 3000

int mode = 0; //0:力が釣り合っているモード, 1:力が発生するモード
float currentX = 0.0f; //x軸の座標
float prevX = 0.0f;

int motor1a = 4; //モータ1正転
int motor1b = 2; //モータ1逆転 //ダミー
int enc1a = 35;   //モータ1エンコーダa相
int enc1b = 39;   //モータ1エンコーダb相

int motor2a = 15; //モータ1正転 //ダミー
int motor2b = 25; //モータ1逆転
int enc2a = 34;   //モータ1エンコーダa相
int enc2b = 36;   //モータ1エンコーダb相

int motor3a = 14; //モータ3正転
int motor3b = 22; //モータ3逆転
int enc3a = 27;   //モータ3エンコーダa相
int enc3b = 26;   //モータ3エンコーダb相

int motor4a = 19; //モータ4正転
int motor4b = 33; //モータ4逆転
int enc4a = 18;   //モータ4エンコーダa相
int enc4b = 19;  

int PWM_HZ = 40000;

Encoder myEnc1(enc1a,enc1b);
Encoder myEnc2(enc2a,enc2b);
Encoder myEnc3(enc3a,enc3b);
Encoder myEnc4(enc4a,enc4b);

/* 関数のプロトタイプ宣言 */
void motorOut(int, float);
int SelectOutChannel(int, float);
int readPos(int);
int ForceFeedback(int, int);

void setup() {
  pinMode(motor1a,OUTPUT);
  pinMode(motor1b,OUTPUT);
  pinMode(motor2a,OUTPUT);
  pinMode(motor2b,OUTPUT);
  pinMode(motor3a,OUTPUT);
  pinMode(motor3b,OUTPUT);
  pinMode(motor4a,OUTPUT);
  pinMode(motor4b,OUTPUT);

  pinMode(enc3a,INPUT);
  pinMode(enc3b,INPUT);
  pinMode(enc4a,INPUT);
  pinMode(enc4b,INPUT);

  ledcSetup(1 ,PWM_HZ,10);
  ledcSetup(2,PWM_HZ,10);
  ledcSetup(3,PWM_HZ,10);
  ledcSetup(4,PWM_HZ,10);
  ledcSetup(5,PWM_HZ,10);
  ledcSetup(6,PWM_HZ,10);
  ledcSetup(7,PWM_HZ,10);
  ledcSetup(8,PWM_HZ,10);
  ledcAttachPin(motor1a,1);
  ledcAttachPin(motor1b,2);
  ledcAttachPin(motor2a,3);
  ledcAttachPin(motor2b,4);
  ledcAttachPin(motor3a,5);
  ledcAttachPin(motor3b,6);
  ledcAttachPin(motor4a,7);
  ledcAttachPin(motor4b,8);

  //pinMode(testDig,OUTPUT);
}

void loop() {
  int force = 0;
  int xpos = readPos(0);

  /*if(xpos > WALLPOS){
    mode = 1;
  }
  else{
    mode = 0;
  }

  if(mode == 1){
    force = ForceFeedback(xpos, WALLPOS);
    motorOut(1,OFFSET_FORECE);
    motorOut(2,-(abs(force)+OFFSET_FORECE));
  }
  else{
    motorOut(1,OFFSET_FORECE);
    motorOut(2,-OFFSET_FORECE);
  }*/

  //dacWrite(testAna,255*xpos/20000);
  //dacWrite(testAna, abs(force));
  //digitalWrite(testDig,mode);
 ledcWrite(1,1000);
 ledcWrite(4,1000);
 ledcWrite(5,1000);
 ledcWrite(8,1000);

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
void motorOut(int motor, float value){
  int ch1 = SelectOutChannel(motor, value);
  int ch2 = SelectOutChannel(motor, -value); //正負が逆転したvalueを与えることで対のチャンネルを得られる
  ledcWrite(ch1,constrain(fabs(value),COUNT_LOW,COUNT_HIGH));
  ledcWrite(ch2,0);
}

/*
出力に使うpwmの出力チャンネルを選ぶ関数
 */
int SelectOutChannel(int motor,float value){
  //正転のとき
  if(value >= 0){
    switch (motor)
    {
      case 1:
        return 1; //pwmの出力チャンネル
        break;
      case 2:
        return 3;
        break;
    
      default:
        return 0;
        break;
    }
  }

  //逆転のとき
  else{
    switch (motor)
    {
      case 1:
        return 2;
        break;
      case 2:
        return 4;
        break;
    
      default:
        return 0;
        break;
    }
  }
}