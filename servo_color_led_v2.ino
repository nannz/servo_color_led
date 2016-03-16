
#if defined(__AVR_ATtiny85__)
#error "This code is for ATmega boards, see other example for ATtiny."
#endif

#define trigPin 12
#define echoPin 13
#include <Adafruit_TiCoServo.h>
int pos = 1000;
int maxSpeed = 100;

Adafruit_TiCoServo myservo; // create servo object to control a servo
int speed2;

int speedPin_M1 = 5;     //M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M1 Direction Control
boolean direct = true;

//const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
//const int analogOutPin = 9; // Analog output pin that the LED is attached to

//int sensorValue = 0;        // value read from the pot
//int outputValue = 0;        // value output to the PWM (analog out)

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define LED_PIN 10
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, LED_PIN, NEO_GRB + NEO_KHZ800);

#define SERVO_PIN    9
#define SERVO_MIN 1000 // 1 ms pulse
#define SERVO_MAX 2000 // 2 ms pulse

#include <Wire.h>
//#include <math.h>
#include <GroveColorSensor.h>

int colorDetect = 0;//red:1 green:2 blue:3

void setup() {                                // Serial initialization

  Serial.begin(9600);
  myservo.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  //myservo.attach(3);// Sets the baud rate to 9600
  Wire.begin();
  //pinMode(speedPin_M1, OUTPUT);
  SensorSetup();

#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  myservo.write(pos);
}

void loop() {
//  carStates();

  strip.clear();
  //rainbow(20);
  colorLED();
  strip.show();


}


void carStop() {               //  Motor Stop
  digitalWrite(speedPin_M2, 0);
  digitalWrite(directionPin_M1, LOW);
  digitalWrite(speedPin_M1, 0);
  digitalWrite(directionPin_M2, LOW);
}

void carAdvance(int leftSpeed, int rightSpeed) {       //Move forward
  analogWrite (speedPin_M2, leftSpeed);             //PWM Speed Control
  digitalWrite(directionPin_M1, HIGH);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}

void carBack(int leftSpeed, int rightSpeed) {     //Move backward
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, LOW);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void carTurnRight(int leftSpeed, int rightSpeed) {           //Turn Left
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, LOW);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}
void carTurnLeft(int leftSpeed, int rightSpeed) {         //Turn Right
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void SensorSetup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int MeasureDistance() {       // a low pull on pin COMP/TRIG  triggering a sensor reading
  long duration;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  long distance = (duration / 2) / 29.1;
  return (int)distance;
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;
  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void colorLED() {
  //get colors from color sensor
  int red, green, blue;
  GroveColorSensor colorSensor;
  colorSensor.ledStatus = 1;              // When turn on the color sensor LED, ledStatus = 1; When turn off the color sensor LED, ledStatus = 0.
  while (1)
  {
    colorSensor.readRGB(&red, &green, &blue);   //Read RGB values to variables.
    delay(5);
    //    Serial.print("The RGB value are: RGB( ");
    Serial.print(red, DEC);
    Serial.print(", ");
    Serial.print(green, DEC);
    Serial.print(", ");
    Serial.println(blue, DEC);
    //Serial.println(" )");
    colorSensor.clearInterrupt();

    if (green >= 50 && red <= 20 && blue <= 25) {
      colorDetect = 2;//green
    } else if (green <= 50 && red >= 45 &&  blue <= 25) {
      colorDetect = 1;//red
    } else if (green <= 58 && red <= 20 && blue >= 50 ) {
      colorDetect = 3; //blue
    } else {
      colorDetect = 0;
    }
    if (colorDetect == 1) {
      pos = 1800;
      colorWipe(strip.Color(255, 0, 0), 10);
    } else if (colorDetect == 2) {
      pos = 1800;
      colorWipe(strip.Color(0, 255, 0), 10);
    } else if (colorDetect == 3) {
      pos = 1800;
      colorWipe(strip.Color(0, 0, 255), 10);
    } else if (colorDetect == 0) {
      pos = 1000;
      rainbow(20);
    }

    myservo.write(pos);//1000-2000
    Serial.println(pos);
    delay(50);

    carStates();
  }
}


void carStates() {
  int actualDistance = MeasureDistance();
  Serial.print(actualDistance);
  Serial.println(" cm");
  //delay(100);
   if (actualDistance <= 40 && actualDistance >= 10) {
    //carTurnRight(255, 255);
    carStop();
    //sweep();

  } else if (actualDistance < 10) {
    carBack(maxSpeed, maxSpeed);
  } else if (actualDistance <= 100 && actualDistance >= 41) {
    speed2 = map(actualDistance, 41, 100, 0, maxSpeed);
    analogWrite (speedPin_M2, speed2);             //PWM Speed Control
    digitalWrite(directionPin_M1, direct);
    analogWrite (speedPin_M1, speed2);
    digitalWrite(directionPin_M2, direct);
    Serial.print("speed2 is ");
    Serial.println(speed2);

  } else {
    carAdvance(maxSpeed, maxSpeed);
  }
}
