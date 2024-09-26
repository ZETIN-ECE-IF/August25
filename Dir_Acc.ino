#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// PS2 컨트롤러용 핀  
#define PS2_DAT        12    
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        13

#define pressures   false
#define rumble      false
#define servo       3

PS2X ps2x; // create PS2 Controller Class

// 뒷바퀴 모터 핀 선언부 (7, 6, 8)
int IN3Pin = 7;
int IN4Pin = 6; 
int EN2Pin = 8; 

int error = 0;
byte type = 0;
byte vibrate = 0;
int up_down=90;
int wheelspeed = 130;

// 이 부분은 servo용
Servo myservo;
byte degree=62;
int cur_degree=62;
void setup(){
  Serial.begin(57600);

  pinMode(IN3Pin, OUTPUT);
  pinMode(IN4Pin, OUTPUT);
  pinMode(EN2Pin, OUTPUT);
  
  myservo.attach(servo); //servo용
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");

  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");

  Serial.print("rumble = ");

  if (rumble)
    Serial.println("true");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  

  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType(); 

  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
   
 cur_degree = myservo.read();
 Serial.println(cur_degree);
 delay(100);
 
}

void loop() {
  if(error != 0) {//skip loop if no controller found
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  }
  
  /* 우리 컨트롤러랑 상관 없는 얘기
  if(type == 2){ //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller  
  }
  혹시 모르니 박제만 해 두자*/

  
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held ");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    /* 안쓰는 컨트롤러 핀
    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.println("Up held this hard: ");
      //Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    } */

    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.println("Right held this hard: ");
      degree += 20;
      //servo모터 돌리기 용으로 넣어놈. 밑에거는 누르는 강도를 나타내는 듯. 확인은 안 해봄;;
      degree = (degree - cur_degree <= 60)? degree : (cur_degree + 60);
      //Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.println("LEFT held this hard: ");
      degree -= 20;
      degree = (cur_degree - degree <= 60)? degree : (cur_degree - 60);
      //Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }

    if(ps2x.Button(PSB_TRIANGLE)){
      digitalWrite(IN3Pin, LOW);
      digitalWrite(IN4Pin, HIGH);
      analogWrite(EN2Pin, wheelspeed);
      Serial.println("pressed Front");
    } 

    else if(ps2x.Button(PSB_CROSS)){
      digitalWrite(IN3Pin, HIGH);
      digitalWrite(IN4Pin, LOW);
      analogWrite(EN2Pin, wheelspeed);
      Serial.println("pressed Back");
    } 

    else {
      digitalWrite(IN3Pin, LOW);
      digitalWrite(IN4Pin, LOW);
      analogWrite(EN2Pin, 50);
    } 


    /* 안 쓰는 컨트롤러 & 함수
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.println("DOWN held this hard: ");
      //Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    } 

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Y pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
      Serial.println("B just pressed");
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      Serial.println("A just changed");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("X just released");     
    
    up_down = map(ps2x.Analog(PSS_LY), 0, 255, -255, 255);
    //PSS_LY = 왼쪽 조이스틱 상하
    //원래 0~255까지 표시해주는 거를 -255~255로 변경
    //값은 밑의 코드 L1, R1 누르면 뜨는거로 확인 가능 
    
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");          //stick_value : 0~255
      Serial.print(up_down, DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
    }     
  }
  */
  }
  
  // 서보모터 돌리기용 0~180도로 범위 정해서 돌아감.
  myservo.write(degree);
  delay(50); 
}
