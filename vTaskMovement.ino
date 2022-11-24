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

int pos = 0;
long posi = 0;

int dos = 0;
long dosi = 0;

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
}

void movement(void * parameter){
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
}

  void Forward(){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }

  void Reverse(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }

  void Left(){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }

  void Right(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }

  void Stop(){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
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
