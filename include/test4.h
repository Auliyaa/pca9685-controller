// source: https://www.aranacorp.com/fr/utilisation-dun-module-pca9685-avec-arduino/
#include <Arduino.h>

#include <Adafruit_PWMServoDriver.h>

// MG90S servo has a 50Hz PWM frequency or 20ms or 20000µs
// The pulse width is located between 400 & 2400µs
// in order to use the setPWM method, we need to find the PWM values for those limits
// The PWM value is between 0 & 4096 so mapping our range gives us:
//
// pwm_min=(400µs / 20000µs) * 4096 = 81.92 -> 90 (rounded for safety)
// pwm_max=(2400µs / 20000µs) * 4096 = 491.52 -> 480 (rounded for safety)
//
// We could also use the writeMicroSeconds method with the following equivalence:
// pca.writeMicroseconds(i,400) <=> pca.setPwm(i,0,90)
// pca.writeMicroseconds(i,2400) <=> pca.setPwm(i,0,480)

#define PWM_FREQ 50
#define US_MIN 400
#define US_MAX 2400
#define PWM_MIN 90
#define PWM_MAX 480

// number of connected servos
// indexing goes from left to right, starting from the V+,VCC,SDA,SCL,DE,GND connectors on the left
#define SERVO_CNT 2

// current microsecond value associated to each servo, initialized in setup()
uint16_t SERVO_MICROSEC[SERVO_CNT] = {};

// PCA board
Adafruit_PWMServoDriver pca(0x40);

void setup() 
{
  Serial.begin(9600);

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PWM_FREQ);

  for(size_t ii=0; ii<SERVO_CNT; ++ii) SERVO_MICROSEC[ii] = US_MIN;

  delay(10);
}

void loop() 
{
    for (uint16_t angle=0; angle <= 180; angle+=20)
    {
        delay(500);
        pca.setPWM(0,0,map(angle,0,180,PWM_MIN,PWM_MAX));
    }
    delay(5000);
    for (uint16_t angle=180; angle >= 0; angle-=20)
    {
        delay(500);
        pca.setPWM(0,0,map(angle,0,180,PWM_MIN,PWM_MAX));
    }
    delay(5000);
}