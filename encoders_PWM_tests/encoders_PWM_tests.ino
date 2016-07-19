
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

volatile int pwmL = 255;
volatile int pwmR = 255;

volatile uint8_t rate_encL;
volatile uint8_t rate_encR;


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
  
  HC05.begin(9600);
  delay(3000);
  while (!HC05.available()){}  
  
    digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, pwmL);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, pwmR);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  HC05.print("PWMs (L, R): "); HC05.print(pwmL); HC05.print("  "); HC05.println(pwmR);
  HC05.print("Rates (L, R): "); HC05.print(rate_encL); HC05.print("  "); HC05.println(rate_encR);
  HC05.print("RPM (L, R): "); HC05.print(rate_encL * 15); HC05.print("  "); HC05.println(rate_encR * 15);
  HC05.println(" ");
  
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
  // if I set tick count to 0 each time I read the sensors, I don't need to remember old tick number
  rate_encL = encoderPosL;
  rate_encR = encoderPosR;

  if (rate_encL == rate_encR)
    ;
    else if ((rate_encL - rate_encR) > 1)
    {
      pwmL--;
      update_speed();
    }
      else if ((rate_encR - rate_encL) > 1)
      {
        pwmR--;
        update_speed;
      }
  
  encoderPosL = 0;
  encoderPosR = 0;
  
}

void update_speed(){

  analogWrite(EN1, pwmL);
    
  analogWrite(EN2, pwmR);

}
