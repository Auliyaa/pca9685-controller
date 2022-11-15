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
uint16_t SERVO_POS[SERVO_CNT] = {};

// PCA board
Adafruit_PWMServoDriver pca(0x40);

void setup() 
{
  Serial.begin(9600);

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(PWM_FREQ);

  delay(10);
}

uint16_t angle_to_us(uint16_t angle)
{
    return map(angle, 0, 180, US_MIN, US_MAX);
}

// move given servo to a specified angle at set speed (safe values for speed_step around [1,8])
void set_angle(uint8_t servo_num, uint16_t angle, uint16_t speed_step)
{
  uint16_t us_src = angle_to_us(SERVO_POS[servo_num]);
  uint16_t us_tgt = angle_to_us(angle);
  if (us_tgt < us_src)
  {
    for (uint16_t us=us_src; us>=us_tgt; us-=speed_step) pca.writeMicroseconds(servo_num,us);
  }
  else
  {
    for (uint16_t us=us_src; us<us_tgt; us+=speed_step) pca.writeMicroseconds(servo_num,us);
  }
  SERVO_POS[servo_num] = us_tgt;
}

void set_angles(uint16_t* tgt_pos, uint16_t speed_step)
{
    static uint16_t src_us[SERVO_CNT];
    static uint16_t tgt_us[SERVO_CNT];

    // map in/out positions to us values
    for (size_t ii=0; ii<SERVO_CNT; ++ii)
    {
      src_us[ii] = angle_to_us(SERVO_POS[ii]);
      tgt_us[ii] = angle_to_us(tgt_pos[ii]);
    }

    bool c = true;
    while (c)
    {
        c=false;
        for (size_t ii=0; ii<SERVO_CNT; ++ii)
        {
          // target position is lower than current and we need at least 1 more step to reach it
          if (src_us[ii] > tgt_us[ii] &&
              src_us[ii]-speed_step >= tgt_us[ii])
          {
            // decrement current us value by 1 step value and update our src value
            c=true;
            src_us[ii]-=speed_step;
            pca.writeMicroseconds(ii,src_us[ii]);
          }
          // target position is higher than current and we need at least 1 more step to reach it
          else if (src_us[ii] > tgt_us[ii] && 
                   src_us+speed_step <= tgt_us[ii])
          {
            // increment current us value by 1 step value and update our src value
            c=true;
            src_us[ii]+=speed_step;
            pca.writeMicroseconds(ii,src_us[ii]);
          }
        }
    }

    memcpy(SERVO_POS, tgt_pos, sizeof(uint16_t)*SERVO_CNT);
}

uint16_t pos=0;
uint16_t SERVO_POS_TGT[SERVO_CNT] = {};
void loop() 
{
  if (pos==0) pos=180;
  else pos=0;

  for (size_t ii=0; ii<SERVO_CNT; ++ii) SERVO_POS_TGT[ii]=pos;

  set_angles(SERVO_POS_TGT, 4);
  delay(1000);
}