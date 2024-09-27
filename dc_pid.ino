#include <Wire.h>
#include <math.h>

#define encoderA 2
#define encoderB 3

#define DX_MIN -30.0
#define DX_MAX 30.0
#define limitPWM 20

#define ENCODER_RATIO 512.f
#define GEAR (76.f / 17.f)
#define PI 3.141593f
#define WHEEL 0.126f
#define RADIAN_PER_METER (GEAR / WHEEL / PI)

#define MOTOR_RESISTER 2.32f // Mexon Motor Motor Resistance
#define MOTOR_KE 0.023405f   // Mexon Motor Motor Inductance

int IN1Pin = 6;
int IN2Pin = 4;
int EN1Pin = 5;
int voltagePin = 0;

const int MPU_ADDR = 0x68;                 // I2C통신을 위한 MPU6050의 주소
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // 가속도(Acceleration)와 자이로(Gyro)

double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;
double averAcX, averAcY, averAcZ; // 자이로 센서 가속도 초기화 변수
double averGyX, averGyY, averGyZ; // 자이로 센서 자이로 초기화 변수
double gyroXfilt = 0;

const double RADIAN_TO_DEGREE = 180 / 3.14159265;
const double DEG_PER_SEC = 32767 / 250; // 1초에 회전하는 각도
const double ALPHA = 1 / (1 + 0.04);
// GyX, GyY, GyZ 값의 범위 : -32768 ~ +32767 (16비트 정수범위)

int16_t encoderCounter = 0; // 엔코더 틱 저장 변수

const float kP = 10.f;   // Gain P
const float kD = 100.f;  // Gain D
const float K1 = 0.3f;   // Gain robot angle
const float K2 = 0.0f;   // Gain K2
const float VMAX = 6.3f; // 최대 회전 속도
const float alpha = 0.4f;

float robotAngle; // 몸체 각도 변수

void setup_mpu();
void get_xyz();
void angle_calc();
void caliSensor();
void setMotorCurrent(float current, float currentVelocity);
void initSettings();
void getData();
void updateEncoder();

void setup()
{
  Serial.begin(1000000);
  initSettings();

  float vWheel = .0f;
  float aWheel = 1.f;

  // Loop 1 - Flywheel PID control
  uint16_t loop1LastTime = micros();
  uint16_t loop1Interval = 10000; // 10ms
  int16_t lastEncoderCounter = encoderCounter;
  float loop1ErrorAccum = 0.f;

  // Loop 2 - Flaywheel acceleration control
  uint16_t loop2LastTime = micros();
  uint16_t loop2Interval = 10000; // 10ms;

  // Loop 3 - Acceleration-body angle control
  uint16_t loop3LastTime = micros();
  uint16_t loop3Interval = 10000; // 10ms;

  uint16_t currentTime;
  while (true)
  {
    // Loop 1
    currentTime = micros();
    if (currentTime - loop1LastTime >= loop1Interval)
    {
      float dt = (float)(currentTime - loop1LastTime) * 0.000001f;
      loop1LastTime += loop1Interval;

      int16_t encoderDiff = encoderCounter - lastEncoderCounter;
      lastEncoderCounter = encoderCounter;

      float vCur = encoderDiff / ENCODER_RATIO / GEAR / dt; // 현재 속도
      float vErr = vWheel - vCur;                           // 기어 목표 속도 - 현재 속도 -> 회전 속도 에러
      loop1ErrorAccum += vErr * dt;
      setMotorCurrent(vErr * kP + loop1ErrorAccum * kD);
      Serial.println(vCur);
    }

    // Loop 2
    currentTime = micros();
    if (currentTime - loop2LastTime >= loop2Interval)
    {
      float dt = (float)(currentTime - loop2LastTime) * 0.000001f;
      loop2LastTime += loop2Interval;

      vWheel = vWheel + aWheel * dt;
      if (vWheel < -7)
        vWheel = -7;
      if (vWheel > 7)
        vWheel = 7;
    }

    // Loop 3
    // currentTime = micros();
    // if (currentTime - loop3LastTime >= loop3Interval)
    // {
    //   get_xyz();
    //   loop3LastTime += loop3Interval;
    //   aWheel = angleFiX * 0.001f;
    // }
  }
}

// 초기화하는 함수
void initSettings()
{
  setup_mpu();
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  pinMode(EN1Pin, OUTPUT);
  analogWrite(EN1Pin, 0);
  digitalWrite(IN1Pin, LOW);
  digitalWrite(IN2Pin, LOW);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
}

// MPU6050 초기화 함수
void setup_mpu()
{
  initSensor();
  caliSensor();
}

// MPU6050에서 가속도, 자이로 값을 읽어오는 함수
void get_xyz()
{
  static uint16_t previousTime = micros();
  uint16_t currentTime = micros();
  float dt = (float)(currentTime - previousTime) * 0.000001f;
  previousTime = currentTime;

  getData();

  angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
  angleAcX *= RADIAN_TO_DEGREE;
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
  angleAcY *= RADIAN_TO_DEGREE;
  // 가속도 센서로는 Z축 회전각 계산 불가함.

  // 가속도 현재 값에서 초기평균값을 빼서 센서값에 대한 보정
  angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt; // 각속도로 변환
  angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
  angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

  // 상보필터 처리를 위한 임시각도 저장
  double angleTmpX = angleFiX + angleGyX * dt;
  double angleTmpY = angleFiY + angleGyY * dt;
  double angleTmpZ = angleFiZ + angleGyZ * dt;

  // (상보필터 값 처리) 임시 각도에 0.96가속도 센서로 얻어진 각도 0.04의 비중을 두어 현재 각도를 구함.
  angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
  angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
  angleFiZ = angleGyZ; // Z축은 자이로 센서만을 이용하열 구함.
  // Serial.print("AngleAcX:");
  // Serial.print(angleAcX);
  // Serial.print("\t FilteredX:");
  // Serial.print(angleFiX);
  // Serial.print("\t AngleAcY:");
  // Serial.print(angleAcY);
  // Serial.print("\t FilteredY:");
  // Serial.println(angleFiY);
  // Serial.print("\t AngleAcZ:");
  // Serial.print(angleGyZ);
  // Serial.print("\t FilteredZ:");
  // Serial.println(angleFiZ);

  // Serial.print("Angle Gyro X:");
  // Serial.print(angleGyX);
  // Serial.print("\t\t Angle Gyro y:");
  // Serial.print(angleGyY);
  // Serial.print("\t\t Angle Gyro Z:");
  // Serial.println(angleGyZ);
}

// 자이로 가속도 센서 초기화 함수
void initSensor()
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // I2C 통신용 어드레스(주소)
  Wire.write(0x6B);                 // MPU6050과 통신을 시작하기 위해서는 0x6B번지에
  Wire.write(0);
  Wire.endTransmission(true);
}

// 가속도 센서 데이터 얻는 함수
void getData()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // AcX 레지스터 위치(주소)를 지칭합니다
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // AcX 주소 이후의 14byte의 데이터를 요청
  AcX = Wire.read() << 8 | Wire.read(); // 두 개의 나뉘어진 바이트를 하나로 이어 붙여서 각 변수에 저장
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// 센서의 초기값을 10회 정도 평균값으로 구하여 저장하는 함수
void caliSensor()
{
  double sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  getData();
  for (int i = 0; i < 10; i++)
  {
    getData();
    sumAcX += AcX;
    sumAcY += AcY;
    sumAcZ += AcZ;
    sumGyX += GyX;
    sumGyY += GyY;
    sumGyZ += GyZ;
    delay(50);
  }
  averAcX = sumAcX / 10;
  averAcY = sumAcY / 10;
  averAcZ = sumAcY / 10;
  averGyX = sumGyX / 10;
  averGyY = sumGyY / 10;
  averGyZ = sumGyZ / 10;

  Serial.println("AcX : ");
  Serial.println(averAcX);
  Serial.println("AcY : ");
  Serial.println(averAcY);
  Serial.println("AcZ : ");
  Serial.println(averAcZ);
}

// 모터 속도 제어 함수
void setMotorCurrent(float current)
{
  float voltage = current * MOTOR_RESISTER; // 전압 계산

  float curVolt = (float)(analogRead(voltagePin) * 5.0 * 6 / 1023); // 현재 전압 측정
  float dutyRatio = abs(voltage) / curVolt;                         // PWM 주기
  int pwm = 255 * dutyRatio;                                        // PWM 절대값 계산
  pwm = constrain(pwm, 0, 255 - limitPWM);                          // PWM 오버플로우 방지
  analogWrite(EN1Pin, pwm);

  // 모터 방향 및 PWM 입력
  if (voltage < 0)
  {
    digitalWrite(IN1Pin, LOW);
    digitalWrite(IN2Pin, HIGH);
    // Serial.print("A");
    // Serial.println(pwm);
  }
  else
  {
    digitalWrite(IN1Pin, HIGH);
    digitalWrite(IN2Pin, LOW);
    // Serial.print("B");
    // Serial.println(pwm);
  }
}

// 엔코더 값 업데이트 함수
void updateEncoder()
{
  // 인터럽트 발생시 실행되는 함수
  static int encoder0PinALast = LOW;
  int encoderPos = digitalRead(encoderA);
  if (encoderPos != encoder0PinALast && encoderPos == HIGH)
  {
    if (digitalRead(encoderB) != encoderPos)
    {
      encoderCounter++;
    }
    else
    {
      encoderCounter--;
    }
  }
  encoder0PinALast = encoderPos;
}
