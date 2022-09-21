int ENCA = 2; // YELLOW
int ENCB = 4;
int ENCC = 3; // WHITE
int ENCD = 7;

int IN1 = 9; // motor 1
int IN2 = 10; // motor 2
int IN3 = 11; // motor 3
int IN4 = 12; // motor 4

int BUTTON1 = 13; 
int BUTTON2 = 8;

int Button1 = 0;
int Button2 = 0;

int PWM1 = 5; // PWM

int pos = 0; // pause ENCA1
long posi = 0; // pause ENCA1

int dos = 0; // pause ENCA2
long dosi = 0; // pause ENCA2

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  
  pinMode(PWM1,OUTPUT);
  
  digitalWrite(ENCA,HIGH);
  digitalWrite(ENCB,HIGH);  
  digitalWrite(ENCC,HIGH);  
  digitalWrite(ENCD,HIGH);
  
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoder1,RISING);

}

void loop() {   
  Button1 = digitalRead(BUTTON1);
  Button2 = digitalRead(BUTTON2);
  
  pos = posi;
  dos = dosi;

  //setForward(IN1,IN2,IN3,IN4,pos,dos,PWM1);
  //setReverse(IN1,IN2,IN3,IN4,pos,dos,PWM1);

  setTurnright(pos,dos,PWM1);
  setTurnleft(pos,dos,PWM1);

  Serial.print("ENC1: ");
  Serial.println(pos);
  Serial.print("ENC2: "); 
  Serial.println(dos);
}
                                                                     

void readEncoder(){            //Pause A
  int b = digitalRead(ENCB);  
  if(b > 0){
    posi++;
  }else{
    posi--;
  }
}

void readEncoder1(){          //Pause B
  int c = digitalRead(ENCD); 
  if(c > 0){
    dosi++;
  }else{
    dosi--;
  }

}

void setForward(int en1, int en2,int pwm1){ // เดินหน้า
  analogWrite(pwm1,255);
  if (Button1 == HIGH){
    digitalWrite(IN1,pwm1);
    digitalWrite(IN2,LOW);  
    digitalWrite(IN3,pwm1);
    digitalWrite(IN4,LOW);

    delay(1000);
    
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
  }
}

void setReverse(int en1, int en2,int pwm1){ //ถอยหลัง;
  analogWrite(pwm1, 255);
  if (Button1 == HIGH){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,pwm1);    
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,pwm1);

    delay(1000);

    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
  }
}

void setTurnleft(int en1, int en2,int pwm1){ //เลี้ยวซ้าย
    analogWrite(pwm1,255);
    if (Button1 == HIGH){
    digitalWrite(IN1,pwm1);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,pwm1);
  }else if (en1 <= -180){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
}
}

void setTurnright(int en1, int en2,int pwm1){ ////เลี้ยวขวา
    analogWrite(pwm1,255);
    if (Button2 == HIGH){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,pwm1);
    digitalWrite(IN3,pwm1);
    digitalWrite(IN4,LOW);
  }else if (en1 >= 180){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
}
}
