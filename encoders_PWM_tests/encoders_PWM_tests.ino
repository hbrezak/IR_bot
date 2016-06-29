/*Čita 40 promjena na CHANGE i na RISING (umjesto 20).
Koristi flag i obican delay koji mi se ne svida jer zaustavlja kod*/

//Zamjenjeni pinovi 6 i 9, tako da mogu koristiti interrupt INT1 koji je 16bitni. EN2 je sada umjesto na pinu D9 na pinu D6 koji je takoder PWM. IN1 je umjesto D6 sada D9 jer
// jer on ne treba PWM (koji se iskljucuje kada se eksplicitno koristi tajmer povezan s njim). Sada mogu implementirati timer interrupt

#define encoderPinR 2
#define encoderPinL 3

unsigned int lastReportedPosR = 1;   // change management
unsigned int lastReportedPosL = 1;   // change management
volatile unsigned int encoderPosR = 0;  // a counter for the dial
volatile unsigned int encoderPosL = 0;  // a counter for the dial
unsigned long previousMillisR = 0;
unsigned long previousMillisL = 0;
uint16_t encoderPosL_old = 0;
uint16_t encoderPosR_old = 0;
int rate_encL = 0;
int rate_encR = 0;
int pwmL = 140;
int pwmR = 140;


//Definicija pinova L293D motor drivera
#define EN1 5    //Enable 1, definiran na PWM pinu - kontrola brzine
#define IN1 9    //Input 1
#define IN2 7    //Input 2

#define EN2 6    //Enable 2, definiran na PWM pinu
#define IN3 8    //Input 3
#define IN4 10   //Input 4

#include <SoftwareSerial.h>
SoftwareSerial HC05(4, 12);
int BluetoothData; // the data given from Computer


void setup() {
  // put your setup code here, to run once:
  pinMode(encoderPinR, INPUT);
  pinMode(encoderPinL, INPUT);
  attachInterrupt(0, doEncoderR, CHANGE);
  attachInterrupt(1, doEncoderL, CHANGE);
  digitalWrite(encoderPinR, HIGH);
  digitalWrite(encoderPinL, HIGH);
  
    //Definicija pinova za upravljanje motorima
  //Lijevi motor
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  analogWrite(EN1, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  //Desni motor
  pinMode(EN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(EN2, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  HC05.begin(9600);
  delay(3000);
  while (!HC05.available()){}

}

void loop() {
  // put your main code here, to run repeatedly:


  if (lastReportedPosL != encoderPosL) {
    HC05.print("Index L:");
    HC05.println(encoderPosL, DEC);
    lastReportedPosL = encoderPosL;
  }
  
   if (lastReportedPosR != encoderPosR) {
    HC05.print("Index R:");
    HC05.println(encoderPosR, DEC);
    lastReportedPosR = encoderPosR;
  }
  
    digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, pwmL);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, pwmR);
  delay(100);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, 0);
  
  rate_encL = encoderPosL - encoderPosL_old;
  rate_encR = encoderPosR - encoderPosR_old;
  
  if ((rate_encL - rate_encR) == 0)
    HC05.println("Same!");
    else if ((rate_encL - rate_encR) > 0)
      pwmL--;
      else pwmR--;
  
  encoderPosL_old = encoderPosL;
  encoderPosR_old = encoderPosR;

  

}

// Interrupt on A changing state
void doEncoderR(){
  if ((unsigned long)(millis() - previousMillisR) >= 7){
    encoderPosR++;
    previousMillisR = millis();
  }
}
  
// Interrupt on A changing state
void doEncoderL(){
  if ((unsigned long)(millis() - previousMillisL) >= 7){
    encoderPosL++;
    previousMillisL = millis();
  }
}
