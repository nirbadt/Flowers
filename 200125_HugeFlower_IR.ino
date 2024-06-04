#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN    5
#define LED_COUNT 24
#define PWM 9
#define INA 6
#define INB 7
#define button 4
#include <math.h>

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

float position;
float speed;
float lastPosition;
float lastSpeed;
int GAP = 0;
int homeDelay = 20000;
int rotDirection = 0;
int pressed = false;
int direction = 1;
int lastDirection = 1;
float LED;
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int buttonState = 0;

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)


  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //https://playground.arduino.cc/Main/TimerPWMCheatsheet/

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  digitalWrite(INA, LOW); // go up to upper limit
  digitalWrite(INB, HIGH);
  analogWrite(PWM, 100);
  Serial.println("going home");
  delay(homeDelay);
  digitalWrite(INA, HIGH); // go up to upper limit
  digitalWrite(INB, LOW);
  delay(1500);

}
void loop() {

  digitalWrite(INA, HIGH); //set direction down
  digitalWrite(INB, LOW);
  for (int d = 90; d < 450; d++) {
    lastPosition = position;
    lastDirection = direction;
    position = sin(DEG_TO_RAD * d);

    analogWrite(PWM, speed);
    LED = map(position * 100, -100, 100, 10, 255);

    if (position > lastPosition) { //direction is upwards
      //Serial.println ("direction changed up");
      direction = 1;
      digitalWrite(INA, LOW); //set direction up
      digitalWrite(INB, HIGH);
      speed = abs(100 * cos(DEG_TO_RAD * d));

    }
    if (position < lastPosition) {  //direction is downwards
      direction = -1;
      digitalWrite(INA, HIGH); //set direction down
      digitalWrite(INB, LOW);
      speed = abs(100 * cos(DEG_TO_RAD * d));

    }

    if (lastDirection != direction) {
      Serial.println("Gap");
      delay(GAP);
    }
    Serial.print("   position   "); //Debuging print
    Serial.print( position);
    Serial.print("   speed   ");
    Serial.print( speed);
    Serial.print("   direction   ");
    Serial.print(direction);
    Serial.print("   LED    ");
    Serial.println(LED);


    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, LED, LED / 5, 0);
    }
    strip.show();

    delay(250);//150
  }
}
