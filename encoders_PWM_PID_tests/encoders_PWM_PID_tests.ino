
// Zamjenjeni pinovi 6 i 9, tako da mogu koristiti interrupt INT1 koji je 16bitni. EN2 je sada umjesto na pinu D9 na pinu D6 koji je takoder PWM. IN1 je umjesto D6 sada D9 jer
// jer on ne treba PWM (koji se iskljucuje kada se eksplicitno koristi tajmer povezan s njim). Sada mogu implementirati timer interrupt

// Dodan timer interrupt koji u ciklusima od 100 ms azurira brzine tj. PWM motora ako je potrebno (ako su dva motora razlicite, u iducoj verziji brzina svakog motora prema vlastitoj
// referenci)

#include <avr/io.h>
#include <avr/interrupt.h>

#define encoderPinR 2
#define encoderPinL 3

volatile unsigned long int previousMillisR = 0;
volatile unsigned long int previousMillisL = 0;

volatile unsigned int encoderPosR = 0;  // a counter for the dial
volatile unsigned int encoderPosL = 0;  // a counter for the dial

volatile int pwmL = 0;
volatile int pwmR = 0;

volatile uint8_t rate_encL;
volatile uint8_t rate_encR;

// PID parameters
const float Kp = 0.5;
const float Ki = 0.2;
const float Kd = 0;

volatile int err_old = 0;
volatile int Err = 0;
volatile int err_dot = 0;

volatile int u_left;
volatile int u_right;
volatile int ref_rpmL = 150;
volatile int ref_rpmR = 150;
volatile int rpmL;
volatile int rpmR;

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
volatile long int count = 0;

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
  
  /*
   cli() - Clear Global Interrupt flag
   sei() - Set Global Interrupt flag
   Frequently, interrupts are being disabled for periods of time in order to perform certain operations without being disturbed
   
   TCCR - Timer/Counter Control Register; registers that hold setup values; in pairs - A and B
   TIMSK - Timer Interrupt Mask Register - used to control which interrupts are "valid" by setting their bits
   TOIE1 - Timer Overflow Interrupt Enable
   OCIE1A - Output Compare Interrupt Enable 1A - if set and if global interrupts are enabled,
         the micro will jump to the Output Compare A interrupt vector upon compare match
   OCR1A - Output Compare register - generate interrupt after the number of clock ticks written to it; CTC bit in TCCR1B must be set 
         if time between is supposed to be equal every time
   */
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
  delay(3000);
  // while (!HC05.available()){}  
  
//    digitalWrite(IN1, HIGH);
//  digitalWrite(IN2, LOW);
//  analogWrite(EN1, pwmL);
//  
//  digitalWrite(IN3, HIGH);
//  digitalWrite(IN4, LOW);
//  analogWrite(EN2, pwmR);

}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  HC05.print("PWMs (L, R): "); HC05.print(pwmL); HC05.print("  "); HC05.println(pwmR);
  HC05.print("Rates (L, R): "); HC05.print(rate_encL); HC05.print("  "); HC05.println(rate_encR);
  HC05.print("RPM (L, R): "); HC05.print(rate_encL * 15); HC05.print("  "); HC05.println(rate_encR * 15);
  HC05.println(" ");*/
  
  //Serial.print("U left: "); Serial.println(u_left);
  Serial.print("Error L: "); Serial.println(ref_rpmL - rpmL);
  Serial.print("Error R: "); Serial.println(ref_rpmR - rpmR);
  Serial.println("");
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
  
  // RPM
  rpmL = 15 * rate_encL; // formula calculated for my sensor and time period of reading sensor data
  rpmR = 15 * rate_encR;

  if (abs(rpmL - ref_rpmL)>2){
      u_left = calc_speed(ref_rpmL - rpmL);
      if (abs(u_left) > 255)
          u_left = sign(u_left) * 255;
      update_speed(); }
  
  if (abs(rpmR - ref_rpmR)>2){
      u_right = calc_speed(ref_rpmR - rpmR);
      if (abs(u_right) > 255)
          u_right = sign(u_right) * 255;
      update_speed(); } 
  
  encoderPosL = 0;
  encoderPosR = 0;  
}

volatile int calc_speed(volatile int err){
  err_dot = err - err_old;
  Err = err + Err;
  err_old = err;
  //Serial.println(err);
 // Serial.print(Kp*err); Serial.print(" ");Serial.print(Ki*Err); Serial.print(" "); Serial.println(Kd * err_dot);
  return (Kp*err + Ki*Err + Kd * err_dot);  
}

void update_speed(){
  
  if (u_left > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  if (u_right > 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);}
    else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    
    analogWrite(EN1, abs(u_left));
    analogWrite(EN2, abs(u_right));

}

int sign(int x){
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);}
