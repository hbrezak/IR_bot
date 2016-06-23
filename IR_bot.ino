#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>


// lijevi senzor
#define echoPinL A4
#define trigPinL A5

// srednji senzor
#define echoPinC A3
#define trigPinC A2

// desni senzor
#define echoPinD A0
#define trigPinD A1

//Definicija pinova L293D motor drivera
#define EN1 5    //Enable 1, definiran na PWM pinu - kontrola brzine
#define IN1 6    //Input 1
#define IN2 7    //Input 2

#define EN2 9    //Enable 2, definiran na PWM pinu
#define IN3 8    //Input 3
#define IN4 10   //Input 4

// define variables
long durationL;
long durationC;
long durationD;
int distanceL;
int distanceC;
int distanceD;

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
FuzzySet* dalekoD = new FuzzySet(50, 100, 400, 400);
FuzzySet* srednjeD = new FuzzySet(5, 50, 50, 100);
FuzzySet* blizuD = new FuzzySet(0, 0, 5, 50);


// Funkcije pripadnosti translacijske brzine
FuzzySet* stani = new FuzzySet(0, 0, 0, 0);
FuzzySet* sporo = new FuzzySet(0, 50, 50, 100);
FuzzySet* umjereno = new FuzzySet(50, 100, 100, 150);
FuzzySet* brzo = new FuzzySet(100, 150, 250, 250);

// Funkcije pripadnosti rotacijske brzine
FuzzySet* brzoD = new FuzzySet(-100, -100, -50, -10); 
FuzzySet* sporoD = new FuzzySet(-50, -20, -20, -5);
FuzzySet* nula = new FuzzySet(-10, 0, 0, 10);
FuzzySet* sporoL = new FuzzySet(5, 20, 20, 50);
FuzzySet* brzoL = new FuzzySet(10, 50, 100, 100);

int left_wheel, right_wheel;

void setup(){
  
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
  
  // Pravilo 1
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiDaleko = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiDaleko->joinWithAND(dalekoL, dalekoS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko->joinWithAND(lijeviDalekoAndSrednjiDaleko, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaBrzoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaBrzoAndRotBrzinaNula->addOutput(brzo);
  thenTransBrzinaBrzoAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo1 = new FuzzyRule(1, ifLijeviDalekoAndSrednjiDalekoAndDesniDaleko, thenTransBrzinaBrzoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo1);
  
  // Pravilo 2
  FuzzyRuleAntecedent* lijeviBlizuAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviBlizuAndSrednjiBlizu->joinWithAND(blizuL, blizuS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu->joinWithAND(lijeviBlizuAndSrednjiBlizu, blizuD);

  FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaStaniAndRotBrzinaNula->addOutput(stani);
  thenTransBrzinaStaniAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo2 = new FuzzyRule(2, ifLijeviBlizuAndSrednjiBlizuAndDesniBlizu, thenTransBrzinaStaniAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo2);
  
  // Pravilo 3
  FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviSrednjeAndSrednjiSrednje->joinWithAND(srednjeL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje->joinWithAND(lijeviSrednjeAndSrednjiSrednje, srednjeD);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo3 = new FuzzyRule(3, ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaUmjerenoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo3);
 
 // Pravilo 4
  //FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiSrednje = new FuzzyRuleAntecedent();
  //lijeviSrednjeAndSrednjiSrednje->joinWithAND(srednjeL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviSrednjeAndSrednjiSrednje, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaSporoD = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaSporoD->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaSporoD->addOutput(sporoD);

  FuzzyRule* pravilo4 = new FuzzyRule(4, ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaUmjerenoAndRotBrzinaSporoD);
  izbjegavanje->addFuzzyRule(pravilo4);
  
  // Pravilo 5
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiSrednje->joinWithAND(dalekoL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje->joinWithAND(lijeviDalekoAndSrednjiSrednje, srednjeD);

  FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaSporoL = new FuzzyRuleConsequent();
  thenTransBrzinaUmjerenoAndRotBrzinaSporoL->addOutput(umjereno);
  thenTransBrzinaUmjerenoAndRotBrzinaSporoL->addOutput(sporoL);

  FuzzyRule* pravilo5 = new FuzzyRule(5, ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaUmjerenoAndRotBrzinaSporoL);
  izbjegavanje->addFuzzyRule(pravilo5);
  
  // Pravilo 6
  //FuzzyRuleAntecedent* lijeviBlizuAndSrednjiBlizu = new FuzzyRuleAntecedent();
  //lijeviBlizuAndSrednjiBlizu->joinWithAND(blizuL, blizuS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiBlizu, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo6 = new FuzzyRule(6, ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo6);  
  
  // Pravilo 7
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiBlizu->joinWithAND(dalekoL, blizuS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiBlizu, blizuD);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoL = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(brzoL);

  FuzzyRule* pravilo7 = new FuzzyRule(7, ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo7);
  
  // Pravilo 8
  //FuzzyRuleAntecedent* lijeviDalekoAndSrednjiSrednje = new FuzzyRuleAntecedent();
  //lijeviDalekoAndSrednjiSrednje->joinWithAND(daleko, srednje);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiSrednje, blizuD);

  //FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoL = new FuzzyRuleConsequent();
  //thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(sporo);
  //thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(brzoL);

  FuzzyRule* pravilo8 = new FuzzyRule(8, ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo8);
  
  // Pravilo 9
  //FuzzyRuleAntecedent* lijeviDalekoAndSrednjiBlizu = new FuzzyRuleAntecedent();
  //lijeviDalekoAndSrednjiBlizu->joinWithAND(dalekoL, blizuS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviDalekoAndSrednjiSrednje, dalekoD);

  //FuzzyRuleConsequent* thenTransBrzinaUmjerenoAndRotBrzinaNula = new FuzzyRuleConsequent();
  //thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(umjereno);
  //thenTransBrzinaUmjerenoAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo9 = new FuzzyRule(9, ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaUmjerenoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo9);
  
  // Pravilo 10
  FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviSrednjeAndSrednjiBlizu->joinWithAND(srednjeL, blizuS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje->joinWithAND(lijeviSrednjeAndSrednjiBlizu, srednjeD);

  //FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  //thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(stani);
  //thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo10 = new FuzzyRule(10, ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo10);  
  
  // Pravilo 11
  FuzzyRuleAntecedent* lijeviBlizuAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviBlizuAndSrednjiSrednje->joinWithAND(blizuL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiSrednje, dalekoD);

  //FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  //thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(sporo);
  //thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo11 = new FuzzyRule(11, ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo11);
  
  
   
  
  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an Output
  pinMode(trigPinC, OUTPUT);
  pinMode(trigPinD, OUTPUT);
  pinMode(echoPinL, INPUT); // Sets the echoPin as an Input
  pinMode(echoPinC, INPUT);
  pinMode(echoPinD, INPUT);
  
  
  Serial.begin(9600); // Starts the serial communication
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
  distanceL = (durationL/2) * 0.034;
  distanceC = (durationC/2) * 0.034;
  distanceD = (durationD/2) * 0.034;
  
  izbjegavanje->setInput(1,distanceL);  
  izbjegavanje->setInput(2,distanceC);  
  izbjegavanje->setInput(3,distanceD);
  
  izbjegavanje->fuzzify();
  
  uint8_t out_transBrzina = izbjegavanje->defuzzify(1);
  uint8_t out_rotBrzina = izbjegavanje->defuzzify(2);
  
  if (out_rotBrzina < 0){
    left_wheel = out_transBrzina + out_rotBrzina;
    right_wheel = out_transBrzina - out_rotBrzina;
  } else if (out_rotBrzina > 0){
    left_wheel = out_transBrzina - out_rotBrzina;
    right_wheel = out_transBrzina + out_rotBrzina;
  } else {
    left_wheel = out_transBrzina;
    right_wheel = out_transBrzina;
  }
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, left_wheel);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, right_wheel);
  delay(100);
  
 
  Serial.print("Inputs: "); Serial.print(distanceL); Serial.print(" "); Serial.print(distanceC); Serial.print(" "); Serial.println(distanceD);
  Serial.print("Output transBrzina: "); Serial.print(out_transBrzina); Serial.print(", rotBrzina: "); Serial.println(out_rotBrzina);
  Serial.print("Left wheel: "); Serial.print(left_wheel); Serial.print(", right_wheel: "); Serial.println(right_wheel);
  Serial.println(" ");
//  Serial.print("1: "); Serial.println(izbjegavanje->isFiredRule(1));
//  
//  Serial.print("2: "); Serial.println(izbjegavanje->isFiredRule(2));
//
//  Serial.print("3: "); Serial.println(izbjegavanje->isFiredRule(3));
//
//  Serial.print("4: "); Serial.println(izbjegavanje->isFiredRule(4));
//
//  Serial.print("5: "); Serial.println(izbjegavanje->isFiredRule(5));
//
//  Serial.print("6: "); Serial.println(izbjegavanje->isFiredRule(6));
//
//  Serial.print("7: "); Serial.println(izbjegavanje->isFiredRule(7));
//
//  Serial.print("8: "); Serial.println(izbjegavanje->isFiredRule(8));
//
//  Serial.print("9: "); Serial.println(izbjegavanje->isFiredRule(9));
//  
//  Serial.print("10: "); Serial.println(izbjegavanje->isFiredRule(10));
//  
//  Serial.print("11: "); Serial.println(izbjegavanje->isFiredRule(11));
  delay(1000);
//  Serial.println(" ");
//
  


  
}

