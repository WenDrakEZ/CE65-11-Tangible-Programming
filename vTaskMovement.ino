//INPUT MOTOR
#define IN1 32
#define IN2 33
#define IN3 4
#define IN4 0

//INPUT PAUSE
const int ENCA = 15;
const int ENCB = 13;
const int ENCC = 12;
const int ENCD = 14;

//Button
int Button = 10;

int pos = 0;
long posi = 0;

int dos = 0;
long dosi = 0;

//PWM
int pval = 255;

TaskHandle_t Move = NULL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoderB,RISING);
  
  xTaskCreate(movement,"taskone",1024,NULL,1,&Move);
  
}

void loop() {
  pos = posi;
  dos = dosi;

  Serial.print("ENA :");
  Serial.println(pos);
  Serial.print("ENB :");
  Serial.println(dos);

  delay(50);
}

void movement(void * parameter){
  pinMode(Button,INPUT);
  
  for (;;){
    if (digitalRead(Button) == HIGH) {
      Forward();
      
      delay(1000);

      Reverse();

      delay(1000);

      Left();

      delay(1000);

      Right();

      delay(1000);
      
      Stop();
     }     
  }
  vTaskDelete(NULL); 
}

  void Forward(){
    analogWrite(IN1,pval);
    analogWrite(IN2,LOW);
    
    analogWrite(IN3,pval);
    analogWrite(IN4,LOW);
  }

  void Reverse(){
    analogWrite(IN1,LOW);
    analogWrite(IN2,pval);
    
    analogWrite(IN3,LOW);
    analogWrite(IN4,pval);
  }

  void Left(){
    analogWrite(IN1,pval);
    analogWrite(IN2,LOW);
    
    analogWrite(IN3,LOW);
    analogWrite(IN4,pval);
  }

  void Right(){
    analogWrite(IN1,LOW);
    analogWrite(IN2,pval);

    analogWrite(IN3,pval);
    analogWrite(IN4,LOW);
  }

  void Stop(){
    analogWrite(IN1,LOW);
    analogWrite(IN2,LOW);
    
    analogWrite(IN3,LOW);
    analogWrite(IN4,LOW);
  }

 void readEncoderA(){            //Pause A
  int b = digitalRead(ENCB);  
  if(b > 0){
    posi++;
  }else{
    posi--;
  }
}

 void readEncoderB(){          //Pause B
  int c = digitalRead(ENCD); 
  if(c > 0){
    dosi++;
  }else{
    dosi--;
  }
}
