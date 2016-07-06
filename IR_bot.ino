// Include Fuzzy library elements
#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

// Include avr libraries needed to access timer interrupts
#include <avr/io.h>
#include <avr/interrupt.h>

// Include library needed to make second serial connection (hardware serial pins used as
// hardware interrupt pins
#include <SoftwareSerial.h>

// Define ultrasonic HC-04 sensor pins
// Left sensor
#define echoPinL A4
#define trigPinL A5

// Middle
#define echoPinC A3
#define trigPinC A2

// Right sensor
#define echoPinD A0
#define trigPinD A1

// Define pins for motor control via L293D H bridge
#define EN1 5    // Enable 1, definiran na PWM pinu - kontrola brzine
#define IN1 9    // Input 1
#define IN2 7    // Input 2

#define EN2 6    // Enable 2, definiran na PWM pinu
#define IN3 8    // Input 3
#define IN4 10   // Input 4

// Define optical encoder signal read pins (hardware interrupt)
#define encoderPinR 2
#define encoderPinL 3

//float R = 0.033; // [m]
//float L = 0.14; // [m]
//const uint8_t maxRPM = 165;

// Init variables for speed read and feedback
volatile unsigned long int previousMillisR = 0;
volatile unsigned long int previousMillisL = 0;

volatile unsigned int encoderPosR = 0;  // a counter for the dial
volatile unsigned int encoderPosL = 0;  // a counter for the dial

volatile uint8_t rate_encL;
volatile uint8_t rate_encR;

// PID parameters and variables
const float Kp = 0.5;
const float Ki = 0.2;
const float Kd = 0;

volatile int err_oldL = 0;
volatile int ErrL = 0;
volatile int err_dotL = 0;
volatile int err_oldR = 0;
volatile int ErrR = 0;
volatile int err_dotR = 0;

volatile int u_left;
volatile int u_right;
volatile int ref_rpmL_d = 0;
volatile int ref_rpmR_d = 0;
volatile int ref_rpmL = 0;
volatile int ref_rpmR = 0;
volatile int rpmL;
volatile int rpmR;

// Define bluetooth serial connection
SoftwareSerial HC05(4, 12);
//int BluetoothData; // the data given from Computer

// Define variables for reading ultrasonic sensor and transforming it to distance
uint16_t durationL;
uint16_t durationC;
uint16_t durationD;
//uint16_t distanceL;
//uint16_t distanceC;
//uint16_t distanceD;

// Instantiating an object of library
Fuzzy* izbjegavanje = new Fuzzy();

// Funkcije pripadnosti ulaza sa senzora
FuzzySet* dalekoL = new FuzzySet(50, 100, 400, 400);
FuzzySet* srednjeL = new FuzzySet(5, 50, 50, 100);
FuzzySet* blizuL = new FuzzySet(0, 0, 5, 50);

// Funkcije pripadnosti ulaza sa senzora
FuzzySet* dalekoS = new FuzzySet(50, 100, 400, 400);
FuzzySet* srednjeS = new FuzzySet(5, 50, 50, 100);
FuzzySet* blizuS = new FuzzySet(0, 0, 5, 50);

// Funkcije pripadnosti ulaza sa senzora
FuzzySet* dalekoD = new FuzzySet(70, 150, 400, 400);
FuzzySet* srednjeD = new FuzzySet(10, 70, 70, 150);
FuzzySet* blizuD = new FuzzySet(0, 0, 10, 70);

// Funkcije pripadnosti translacijske brzine
FuzzySet* stani = new FuzzySet(0, 0, 0, 0);
FuzzySet* sporo = new FuzzySet(45, 45, 75, 90);
FuzzySet* umjereno = new FuzzySet(75, 90, 90, 105);
FuzzySet* brzo = new FuzzySet(90, 105, 150, 150);

// Funkcije pripadnosti rotacijske brzine
FuzzySet* brzoD = new FuzzySet(-60, -60, -45, -30); 
FuzzySet* sporoD = new FuzzySet(-45, -30, -30, -10);
FuzzySet* nula = new FuzzySet(-15, 0, 0, 15);
FuzzySet* sporoL = new FuzzySet(10, 30, 30, 45);
FuzzySet* brzoL = new FuzzySet(30, 45, 60, 60);

void setup(){

  // Set up encoders and their interrupts
  pinMode(encoderPinR, INPUT);
  pinMode(encoderPinL, INPUT);
  attachInterrupt(0, doEncoderR, CHANGE);
  attachInterrupt(1, doEncoderL, CHANGE);
  digitalWrite(encoderPinR, HIGH);
  digitalWrite(encoderPinL, HIGH);

  // Set up ultrasonic sensor pins
  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an Output
  pinMode(trigPinC, OUTPUT);
  pinMode(trigPinD, OUTPUT);
  pinMode(echoPinL, INPUT); // Sets the echoPin as an Input
  pinMode(echoPinC, INPUT);
  pinMode(echoPinD, INPUT);

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

  // FuzzyInput lijevi senzor
  FuzzyInput* lijevi = new FuzzyInput(1);
  lijevi->addFuzzySet(dalekoL);
  lijevi->addFuzzySet(srednjeL);
  lijevi->addFuzzySet(blizuL);

  izbjegavanje->addFuzzyInput(lijevi);

  // FuzzyInput srednji senzor
  FuzzyInput* centar = new FuzzyInput(2);
  centar->addFuzzySet(dalekoS);
  centar->addFuzzySet(srednjeS);
  centar->addFuzzySet(blizuS);

  izbjegavanje->addFuzzyInput(centar);

  // FuzzyInput desni senzor
  FuzzyInput* desni = new FuzzyInput(3);
  desni->addFuzzySet(dalekoD);
  desni->addFuzzySet(srednjeD);
  desni->addFuzzySet(blizuD);

  izbjegavanje->addFuzzyInput(desni);

  // FuzzyOutput translacijska brzina
  FuzzyOutput* transBrzina = new FuzzyOutput(1);

  transBrzina->addFuzzySet(stani);  
  transBrzina->addFuzzySet(sporo);
  transBrzina->addFuzzySet(umjereno);
  transBrzina->addFuzzySet(brzo);

  izbjegavanje->addFuzzyOutput(transBrzina);

  // FuzzyOutput rotacijska brzina
  FuzzyOutput* rotBrzina = new FuzzyOutput(2);

  rotBrzina->addFuzzySet(brzoD);
  rotBrzina->addFuzzySet(sporoD);
  rotBrzina->addFuzzySet(nula);
  rotBrzina->addFuzzySet(sporoL);
  rotBrzina->addFuzzySet(brzoL);

  izbjegavanje->addFuzzyOutput(rotBrzina);

  // Definiranje premisa
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiDaleko = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiDaleko->joinWithAND(dalekoL, dalekoS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko->joinWithAND(lijeviDalekoAndSrednjiDaleko, dalekoD);

  FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviSrednjeAndSrednjiSrednje->joinWithAND(srednjeL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje->joinWithAND(lijeviSrednjeAndSrednjiSrednje, srednjeD);

  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviSrednjeAndSrednjiSrednje, dalekoD);

  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiSrednje->joinWithAND(dalekoL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje->joinWithAND(lijeviDalekoAndSrednjiSrednje, srednjeD);

  FuzzyRuleAntecedent* lijeviBlizuAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviBlizuAndSrednjiBlizu->joinWithAND(blizuL, blizuS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiBlizu, dalekoD);

  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiBlizu->joinWithAND(dalekoL, blizuS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiBlizu, blizuD);

  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiSrednje, blizuD);

  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviDalekoAndSrednjiSrednje, dalekoD);

  FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviSrednjeAndSrednjiBlizu->joinWithAND(srednjeL, blizuS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje->joinWithAND(lijeviSrednjeAndSrednjiBlizu, srednjeD);

  FuzzyRuleAntecedent* lijeviBlizuAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviBlizuAndSrednjiSrednje->joinWithAND(blizuL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiSrednje, dalekoD);

  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu->joinWithAND(lijeviBlizuAndSrednjiBlizu, blizuD);

  // Definiranje zakljucaka
  FuzzyRuleConsequent* thenTransBrzinaBrzoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaBrzoAndRotBrzinaNula->addOutput(brzo);
  thenTransBrzinaBrzoAndRotBrzinaNula->addOutput(nula);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(nula);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaSporoD = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaSporoD->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaSporoD->addOutput(sporoD);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaSporoL = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaSporoL->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaSporoL->addOutput(sporoL);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoL = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(brzoL);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaNula->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaNula->addOutput(nula);

  FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaStaniAndRotBrzinaNula->addOutput(stani);
  thenTransBrzinaStaniAndRotBrzinaNula->addOutput(nula);

  // Definiranje pravila
  // Pravilo 1
  FuzzyRule* pravilo1 = new FuzzyRule(1, ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko, thenTransBrzinaBrzoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo1);

  // Pravilo 3
  FuzzyRule* pravilo3 = new FuzzyRule(3, ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaUmjerenoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo3);

  // Pravilo 4
  FuzzyRule* pravilo4 = new FuzzyRule(4, ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaUmjerenoAndRotBrzinaSporoD);
  izbjegavanje->addFuzzyRule(pravilo4);

  // Pravilo 5
  FuzzyRule* pravilo5 = new FuzzyRule(5, ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaUmjerenoAndRotBrzinaSporoL);
  izbjegavanje->addFuzzyRule(pravilo5);

  // Pravilo 6
  FuzzyRule* pravilo6 = new FuzzyRule(6, ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo6);  

  // Pravilo 7
  FuzzyRule* pravilo7 = new FuzzyRule(7, ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo7);

  // Pravilo 8
  FuzzyRule* pravilo8 = new FuzzyRule(8, ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo8);

  // Pravilo 9
  FuzzyRule* pravilo9 = new FuzzyRule(9, ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaUmjerenoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo9);

  // Pravilo 10
  FuzzyRule* pravilo10 = new FuzzyRule(10, ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje, thenTransBrzinaSporoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo10);  

  // Pravilo 11
  FuzzyRule* pravilo11 = new FuzzyRule(11, ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo11);

  // Pravilo 2
  FuzzyRule* pravilo2 = new FuzzyRule(2, ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu, thenTransBrzinaStaniAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo2);

  // Setting timer interrupt
  cli();
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 6249;

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A);
  sei(); 

  Serial.begin(9600);
  Serial.println("Starting program in 5 seconds...");
  delay(5000);
}

void loop(){  

  // clear trigPins
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinC, LOW);
  digitalWrite(trigPinD, LOW);
  delayMicroseconds(2);

  // set trigPin HIGH for 10 microsec
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  // read echoPin to recieve sound wave travel time in microsec
  durationL = pulseIn(echoPinL, HIGH);

  // set trigPin HIGH for 10 microsec
  digitalWrite(trigPinC, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinC, LOW);

  // read echoPin to recieve sound wave travel time in microsec
  durationC = pulseIn(echoPinC, HIGH);

  // set trigPin HIGH for 10 microsec
  digitalWrite(trigPinD, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinD, LOW);

  // read echoPin to recieve sound wave travel time in microsec
  durationD = pulseIn(echoPinD, HIGH); 

  // calculate the distance
 // distanceL = (durationL/2) * 0.034;
 // distanceC = (durationC/2) * 0.034;
 // distanceD = (durationD/2) * 0.034;

  izbjegavanje->setInput(1,durationL * 0.017);  
  izbjegavanje->setInput(2,durationC * 0.017);  
  izbjegavanje->setInput(3,durationD * 0.017);

  izbjegavanje->fuzzify();

  int8_t out_transBrzina = izbjegavanje->defuzzify(1);
  int8_t out_rotBrzina = izbjegavanje->defuzzify(2);

  ref_rpmL_d = out_transBrzina - 2.12 * out_rotBrzina; // 2.12 = L/(2*R) [m]
  ref_rpmR_d = out_transBrzina + 2.12 * out_rotBrzina;

  cli();
  // 165 = maxRPM for IR BOT
  if (max(ref_rpmL_d, ref_rpmR_d) > 165)
    ref_rpmL = ref_rpmL_d - (max(ref_rpmL_d, ref_rpmR_d) - 165);
  else if (min(ref_rpmL_d, ref_rpmR_d) < -165)
    ref_rpmL = ref_rpmL_d - (min(ref_rpmL_d, ref_rpmR_d) + 165);
  else ref_rpmL = ref_rpmL_d;

  if (max(ref_rpmL_d, ref_rpmR_d) > 165)
    ref_rpmR = ref_rpmR_d - (max(ref_rpmL_d, ref_rpmR_d) - 165);
  else if (min(ref_rpmL_d, ref_rpmR_d) < -165)
    ref_rpmR = ref_rpmR_d - (min(ref_rpmL_d, ref_rpmR_d) + 165);
  else ref_rpmR = ref_rpmR_d;
  sei();

//  Serial.print("Inputs: "); 
//  Serial.print(distanceL); 
//  Serial.print(" "); 
//  Serial.print(distanceC); 
//  Serial.print(" "); 
//  Serial.println(distanceD);
  //  Serial.print("transB, rotB: "); Serial.print(out_transBrzina); Serial.print(" "); Serial.println(out_rotBrzina);
  //  Serial.print("RPM desired L, R: "); Serial.print(ref_rpmL_d); Serial.print(" "); Serial.println(ref_rpmR_d);
  //  Serial.print("RPM L, R: "); Serial.print(ref_rpmL); Serial.print(" "); Serial.println(ref_rpmR);
  //  HC05.print("Left wheel: "); HC05.print(left_wheel); HC05.print(", right_wheel: "); HC05.println(right_wheel);
  //Serial.println(" ");
}

// Interrupt on A changing state
void doEncoderR(){
  if ((unsigned long)(millis() - previousMillisR) >= 2){
    encoderPosR++;
    previousMillisR = millis();
  }
}

// Interrupt on A changing state
void doEncoderL(){
  if ((unsigned long)(millis() - previousMillisL) >= 2){
    encoderPosL++;
    previousMillisL = millis();
  }
}

ISR(TIMER1_COMPA_vect)
{
  // delta tick
  rate_encL = encoderPosL;
  rate_encR = encoderPosR;

  if ((ref_rpmL == 0)&&(ref_rpmR==0)){
    u_left = 0;
    u_right = 0;
    update_speedL();
    update_speedR();
  }
  else {
    // RPM
    rpmL = sign(ref_rpmL) * 15 * rate_encL; // formula calculated for my sensor and time period of reading sensor data
    rpmR = sign(ref_rpmR) * 15 * rate_encR;

    if (abs(rpmL - ref_rpmL)>2){
      u_left = calc_speedL(ref_rpmL - rpmL);
      if (abs(u_left) > 255)
        u_left = sign(u_left) * 255;
      update_speedL(); 
    }

    if (abs(rpmR - ref_rpmR)>2){
      u_right = calc_speedR(ref_rpmR - rpmR);
      if (abs(u_right) > 255)
        u_right = sign(u_right) * 255;
      update_speedR(); 
    } 
  }
  encoderPosL = 0;
  encoderPosR = 0;  
}

volatile int calc_speedL(volatile int err){
  err_dotL = err - err_oldL;
  ErrL = err + ErrL;
  err_oldL = err;
  //Serial.println(err);
  // Serial.print(Kp*err); Serial.print(" ");Serial.print(Ki*Err); Serial.print(" "); Serial.println(Kd * err_dot);
  return (Kp*err + Ki*ErrL + Kd * err_dotL); 
}

volatile int calc_speedR(volatile int err){
  err_dotR = err - err_oldR;
  ErrR = err + ErrR;
  err_oldR = err;
  //Serial.println(err);
  // Serial.print(Kp*err); Serial.print(" ");Serial.print(Ki*Err); Serial.print(" "); Serial.println(Kd * err_dot);
  return (Kp*err + Ki*ErrR + Kd * err_dotR);  
}

void update_speedL(){  
  if (u_left >= 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(EN1, abs(u_left));
}

void update_speedR(){  
  if (u_right >= 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } 
  analogWrite(EN2, abs(u_right));
}

int sign(int x){
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}


