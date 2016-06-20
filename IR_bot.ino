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
const int trigPinL = 8;
const int echoPinL = 9;

// srednji senzor
const int trigPinC = 5;
const int echoPinC = 6;

// desni senzor
const int trigPinD = 2;
const int echoPinD = 3;

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
FuzzySet* srednjeL = new FuzzySet(15, 50, 50, 100);
FuzzySet* blizuL = new FuzzySet(0, 0, 15, 50);

// Funkcije pripadnosti ulaza sa senzora
FuzzySet* dalekoS = new FuzzySet(50, 100, 400, 400);
FuzzySet* srednjeS = new FuzzySet(15, 50, 50, 100);
FuzzySet* blizuS = new FuzzySet(0, 0, 15, 50);

// Funkcije pripadnosti ulaza sa senzora
FuzzySet* dalekoD = new FuzzySet(50, 100, 400, 400);
FuzzySet* srednjeD = new FuzzySet(15, 50, 50, 100);
FuzzySet* blizuD = new FuzzySet(0, 0, 15, 50);


// Funkcije pripadnosti translacijske brzine
FuzzySet* stani = new FuzzySet(0, 0, 0.25, 0.5);
FuzzySet* sporo = new FuzzySet(0, 0.5, 0.5, 1);
FuzzySet* brzo = new FuzzySet(0.5, 1, 4, 4);

// Funkcije pripadnosti rotacijske brzine
FuzzySet* brzoD = new FuzzySet(-8, -8, -3, -1.5); 
FuzzySet* sporoD = new FuzzySet(-3, -1.5, -1.5, 0);
FuzzySet* nula = new FuzzySet(-1, 0, 0, 1);
FuzzySet* sporoL = new FuzzySet(0, 1.5, 1.5, 3);
FuzzySet* brzoL = new FuzzySet(1.5, 3, 8, 8);


void setup(){

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

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaNula = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaNula->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo3 = new FuzzyRule(3, ifLijeviSrednjeAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaSporoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo3);
 
 // Pravilo 4
  //FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiSrednje = new FuzzyRuleAntecedent();
  //lijeviSrednjeAndSrednjiSrednje->joinWithAND(srednjeL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviSrednjeAndSrednjiSrednje, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaBrzoAndRotBrzinaSporoD = new FuzzyRuleConsequent();
  thenTransBrzinaBrzoAndRotBrzinaSporoD->addOutput(brzo);
  thenTransBrzinaBrzoAndRotBrzinaSporoD->addOutput(sporoD);

  FuzzyRule* pravilo4 = new FuzzyRule(4, ifLijeviSrednjeAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaBrzoAndRotBrzinaSporoD);
  izbjegavanje->addFuzzyRule(pravilo4);
  
  // Pravilo 5
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiSrednje->joinWithAND(dalekoL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje->joinWithAND(lijeviDalekoAndSrednjiSrednje, srednjeD);

  FuzzyRuleConsequent* thenTransBrzinaBrzoAndRotBrzinaSporoL = new FuzzyRuleConsequent();
  thenTransBrzinaBrzoAndRotBrzinaSporoL->addOutput(brzo);
  thenTransBrzinaBrzoAndRotBrzinaSporoL->addOutput(sporoL);

  FuzzyRule* pravilo5 = new FuzzyRule(5, ifLijeviDalekoAndSrednjiSrednjeAndDesniSrednje, thenTransBrzinaBrzoAndRotBrzinaSporoL);
  izbjegavanje->addFuzzyRule(pravilo5);
  
  // Pravilo 6
  //FuzzyRuleAntecedent* lijeviBlizuAndSrednjiBlizu = new FuzzyRuleAntecedent();
  //lijeviBlizuAndSrednjiBlizu->joinWithAND(blizuL, blizuS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiBlizu, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(stani);
  thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo6 = new FuzzyRule(6, ifLijeviBlizuAndSrednjiBlizuAndDesniDaleko, thenTransBrzinaStaniAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo6);  
  
  // Pravilo 7
  FuzzyRuleAntecedent* lijeviDalekoAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviDalekoAndSrednjiBlizu->joinWithAND(dalekoL, blizuS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiBlizu, blizuD);

  FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaBrzoL = new FuzzyRuleConsequent();
  thenTransBrzinaStaniAndRotBrzinaBrzoL->addOutput(stani);
  thenTransBrzinaStaniAndRotBrzinaBrzoL->addOutput(brzoL);

  FuzzyRule* pravilo7 = new FuzzyRule(7, ifLijeviDalekoAndSrednjiBlizuAndDesniBlizu, thenTransBrzinaStaniAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo7);
  
  // Pravilo 8
  //FuzzyRuleAntecedent* lijeviDalekoAndSrednjiSrednje = new FuzzyRuleAntecedent();
  //lijeviDalekoAndSrednjiSrednje->joinWithAND(daleko, srednje);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu->joinWithAND(lijeviDalekoAndSrednjiSrednje, blizuD);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoL = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoL->addOutput(brzoL);

  FuzzyRule* pravilo8 = new FuzzyRule(8, ifLijeviDalekoAndSrednjiSrednjeAndDesniBlizu, thenTransBrzinaSporoAndRotBrzinaBrzoL);
  izbjegavanje->addFuzzyRule(pravilo8);
  
  // Pravilo 9
  //FuzzyRuleAntecedent* lijeviDalekoAndSrednjiBlizu = new FuzzyRuleAntecedent();
  //lijeviDalekoAndSrednjiBlizu->joinWithAND(dalekoL, blizuS);
  FuzzyRuleAntecedent* ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviDalekoAndSrednjiSrednje, dalekoD);

  //FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaNula = new FuzzyRuleConsequent();
  //thenTransBrzinaSporoAndRotBrzinaNula->addOutput(sporo);
  //thenTransBrzinaSporoAndRotBrzinaNula->addOutput(nula);

  FuzzyRule* pravilo9 = new FuzzyRule(9, ifLijeviDalekoAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaNula);
  izbjegavanje->addFuzzyRule(pravilo9);
  
  // Pravilo 10
  FuzzyRuleAntecedent* lijeviSrednjeAndSrednjiBlizu = new FuzzyRuleAntecedent();
  lijeviSrednjeAndSrednjiBlizu->joinWithAND(srednjeL, blizuS);
  FuzzyRuleAntecedent* ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje = new FuzzyRuleAntecedent();
  ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje->joinWithAND(lijeviSrednjeAndSrednjiBlizu, srednjeD);

  //FuzzyRuleConsequent* thenTransBrzinaStaniAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  //thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(stani);
  //thenTransBrzinaStaniAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo10 = new FuzzyRule(10, ifLijeviSrednjeAndSrednjiBlizuAndDesniSrednje, thenTransBrzinaStaniAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo10);  
  
  // Pravilo 11
  FuzzyRuleAntecedent* lijeviBlizuAndSrednjiSrednje = new FuzzyRuleAntecedent();
  lijeviBlizuAndSrednjiSrednje->joinWithAND(blizuL, srednjeS);
  FuzzyRuleAntecedent* ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko = new FuzzyRuleAntecedent();
  ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko->joinWithAND(lijeviBlizuAndSrednjiSrednje, dalekoD);

  FuzzyRuleConsequent* thenTransBrzinaSporoAndRotBrzinaBrzoD = new FuzzyRuleConsequent();
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(sporo);
  thenTransBrzinaSporoAndRotBrzinaBrzoD->addOutput(brzoD);

  FuzzyRule* pravilo11 = new FuzzyRule(11, ifLijeviBlizuAndSrednjiSrednjeAndDesniDaleko, thenTransBrzinaSporoAndRotBrzinaBrzoD);
  izbjegavanje->addFuzzyRule(pravilo11);
  
  
   
  
  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an Output
  pinMode(trigPinC, OUTPUT);
  pinMode(trigPinD, OUTPUT);
  pinMode(echoPinL, INPUT); // Sets the echoPin as an Input
  pinMode(echoPinC, INPUT);
  pinMode(echoPinD, INPUT);
  
  Serial.begin(9600); // Starts the serial communication

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
  
  izbjegavanje->setInput(1,150);  
  izbjegavanje->setInput(2,50);  
  izbjegavanje->setInput(3,50);
  
  izbjegavanje->fuzzify();
Serial.print(brzoD->getPertinence());Serial.print(" ");Serial.println(brzoL->getPertinence());
  
  float out_transBrzina = izbjegavanje->defuzzify(1);
  float out_rotBrzina = izbjegavanje->defuzzify(2);
  
 
//  Serial.print("Inputs: "); Serial.print(distanceL); Serial.print(" "); Serial.print(distanceC); Serial.print(" "); Serial.println(distanceD);
  Serial.print("Output transBrzina: "); Serial.print(out_transBrzina); Serial.print(", rotBrzina: "); Serial.println(out_rotBrzina);
  Serial.print("1: "); Serial.println(izbjegavanje->isFiredRule(1));
  
  Serial.print("2: "); Serial.println(izbjegavanje->isFiredRule(2));

  Serial.print("3: "); Serial.println(izbjegavanje->isFiredRule(3));

  Serial.print("4: "); Serial.println(izbjegavanje->isFiredRule(4));

  Serial.print("5: "); Serial.println(izbjegavanje->isFiredRule(5));

  Serial.print("6: "); Serial.println(izbjegavanje->isFiredRule(6));

  Serial.print("7: "); Serial.println(izbjegavanje->isFiredRule(7));

  Serial.print("8: "); Serial.println(izbjegavanje->isFiredRule(8));

  Serial.print("9: "); Serial.println(izbjegavanje->isFiredRule(9));
  
  Serial.print("10: "); Serial.println(izbjegavanje->isFiredRule(10));
  
  Serial.print("11: "); Serial.println(izbjegavanje->isFiredRule(11));
  Serial.println(" ");

  delay(2000);
  
}

