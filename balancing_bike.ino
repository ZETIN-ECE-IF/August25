#include <Wire.h>
#include <math.h>

// 자이로 모터 핀 선언부 (3,4,5)
#define DX_MAX 60.0

#define adt            0.001

#define ENCODER_RATIO	2048
#define GEAR 					5
#define PI						3.141593
#define WHEEL					0.037
#define RADIAN_PER_METER		(GEAR / WHEEL / PI)

#define GAIN_P					35
#define GAIN_D					0.001

#define MOTOR_RESISTER			2.32
#define MOTOR_KE				0.023405

int IN1Pin = 5; //5번 핀
int IN2Pin = 6; //6번 핀
int EN1Pin = 3; //3번 핀

int voltPin = 0;

int encoder0PinA = 7;
int encoder0PinB = 8;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;

float v = 0;
float posCmd = 0;
float posEnc = 0;
float prePosEnc = 0;
int preEncVal = 0;
int curTick = 0;

double custom_constrain(double x){
  if(x < -60.0) return -60.0;
  if(x > 60.0)  return 60.0;

  return x;
}

void setup(void) {
  Serial.begin(115200);


  /*
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  */

  setup_mpu();

  // Serial.println("MPU6050 Found!");

  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // Serial.println("");
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);  

  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  pinMode(EN1Pin, OUTPUT);

  analogWrite(EN1Pin, 0);
  delay(10);

  digitalWrite(IN1Pin, LOW);
  digitalWrite(IN2Pin, HIGH);
  analogWrite(EN1Pin, 255);
}

int get_encoder(){
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  //  Serial.println (encoder0Pos);
  }
  encoder0PinALast = n;
  return encoder0Pos;
}

void loop() {
  get_xyz();

  float DX = custom_constrain(getX());
  Serial.println(DX);
  //Serial.println(DX);

  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  float vCmd = v * RADIAN_PER_METER;
  int curEncVal = get_encoder();
  posCmd += vCmd * adt;
  float vEnc = (curEncVal - preEncVal) / adt;
  posEnc += vEnc * adt;

  float vErr = vEnc - vCmd;
  float posErr = posEnc - posCmd;

  float current = -(posErr * GAIN_P + vErr * GAIN_D);
  float voltage = current * MOTOR_RESISTER + vEnc * MOTOR_KE;

  float curVolt = 24; //(float)(analogRead(voltPin) * 3.3 * 21 / 1023);

  float dutyRatio = voltage / curVolt;

  int pwm = 255 * abs(dutyRatio);

	pwm = min(pwm, 255);

  //Serial.println(curEncVal);
	curTick += curEncVal - preEncVal;
	
  preEncVal = curEncVal;

  if(DX > 0){ // 좌측 회전
    digitalWrite(IN1Pin, HIGH);
    digitalWrite(IN2Pin, LOW);
  }
  
  if(DX < 0){ // 우측 회전
    digitalWrite(IN1Pin, LOW);
    digitalWrite(IN2Pin, HIGH);
  }

  analogWrite(EN1Pin, pwm);
  delay(10);
}
