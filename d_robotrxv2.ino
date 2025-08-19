//robot
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
SoftwareSerial BTSerial(7, 8);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define arm_speed 4095       //12bit pwm
#define encoder_read_late 1  //ms
#define one_lotate_encoder_step 341.2
#define Kp_tune 1.0
#define Kd_tune 1.0


#define Buzzer 3
#define M1IN1 0
#define M1IN2 1
#define M1IN3 2
#define M1IN4 3
#define M2IN1 4
#define M2IN2 5
#define M2IN3 6
#define M2IN4 7
#define M3IN1 8
#define M3IN2 9
#define M3IN3 10
#define M3IN4 11
#define M1eA 0
#define M1eB 1
#define M2eA 2
#define M2eB 4
#define M3eA 5
#define M3eB 12

uint8_t buf[5];
int xbuf = 0, ybuf = 0, parity_tmp = 0, switch_tmp = 0, arm_switch = 0, LED_pwm = 0, LED_count = 0, m1direction = 0, m2direction = 0, m3direction = 0;
volatile int m1counter = 0, m2counter = 0, m3counter = 0;
const int LEDpins[8] = { 12, 13, 14, 15, 6, 9, 10, 11 };
long speed = 0, w1 = 0, w2 = 0, w3 = 0, before_time = 0, current_time = 0, error_before_time = 0, error_now_time = 0;
volatile long m1_deltat = 0, m2_deltat = 0, m3_deltat = 0, m1_before_time = 0, m2_before_time = 0, m3_before_time = 0;
float cood_dig = 0, m1rpm = 0, m2rpm = 0, m3rpm = 0, m1coefficient = 0, m2coefficient = 0, m3coefficient = 0, m1section = 0, m2section = 0, m3section = 0;
float m1_target_rpm = 0, m2_target_rpm = 0, m3_target_rpm = 0, m1Kp = 0, m2Kp = 0, m3Kp = 0, m1Kd = 0, m2Kd = 0, m3Kd = 0;
float old_rpm_alpha = 0.2, before_m1error = 0, before_m2error = 0, before_m3error = 0;
bool switch_buf[5], arm_direction, lighting_flag, w1dir, w2dir, w3dir;
const bool LEDpin_location[8] = { false, false, false, false, true, true, true, true };

void rn42command(char* command) {
  BTSerial.print("$$$");
  delay(100);
  BTSerial.println(command);
  delay(100);
  BTSerial.println("R,1");
}

void motormove(int IN1, int IN2, int speed, bool direction) {
  if (direction) {
    pwm.setPWM(IN1, 0, speed);
    pwm.setPWM(IN2, 0, 0);
  } else {
    pwm.setPWM(IN1, 0, 0);
    pwm.setPWM(IN2, 0, speed);
  }
}

void kasuko() {
  tone(Buzzer, 2093, 100);
  delay(500);
  tone(Buzzer, 2093, 100);
  delay(100);
  tone(Buzzer, 2637, 100);
  delay(100);
  tone(Buzzer, 3136, 100);
}

void lighting() {
  lighting_flag = true;
}

void readrpm() {
  unsigned long now = micros();
  int w1_bias = 1, w2_bias = 1, w3_bias = 1;

  if (now - m1_before_time > 2000000UL) w1_bias = 0;
  if (now - m2_before_time > 2000000UL) w2_bias = 0;
  if (now - m3_before_time > 2000000UL) w3_bias = 0;

  noInterrupts();
  long m1t = m1_deltat;
  int dir1 = m1direction;
  long m2t = m2_deltat;
  int dir2 = m2direction;
  long m3t = m3_deltat;
  int dir3 = m3direction;
  interrupts();

  if (m1t > 0) { m1rpm = (1000000 / m1t / one_lotate_encoder_step * (float)dir1 * 60.0f * w1_bias) * (1 - old_rpm_alpha) + m1rpm * old_rpm_alpha; }
  if (m2t > 0) { m2rpm = (1000000 / m2t / one_lotate_encoder_step * (float)dir2 * 60.0f * w2_bias) * (1 - old_rpm_alpha) + m2rpm * old_rpm_alpha; }
  if (m3t > 0) { m3rpm = (1000000 / m3t / one_lotate_encoder_step * (float)dir3 * 60.0f * w3_bias) * (1 - old_rpm_alpha) + m3rpm * old_rpm_alpha; }
}

void M1counter() {
  unsigned long now = micros();
  if (digitalRead(M1eB) == 0) m1direction = 1;
  else m1direction = -1;
  m1_deltat = now - m1_before_time;
  m1_before_time = now;
}

void M2counter() {
  unsigned long now = micros();
  if (digitalRead(M2eB) == 0) m2direction = 1;
  else m2direction = -1;
  m2_deltat = now - m2_before_time;
  m2_before_time = now;
}

void M3counter() {
  unsigned long now = micros();
  if (digitalRead(M3eB) == 0) m3direction = 1;
  else m3direction = -1;
  m3_deltat = now - m3_before_time;
  m3_before_time = now;
}

void setup() {
  //initialize
  noInterrupts();
  BTSerial.begin(115200);
  Serial.begin(115200);  //debug
  pwm.begin();
  pwm.setPWMFreq(1526);
  rn42command("SY,0000");
  MsTimer2::set(5, lighting);
  MsTimer2::start();

  attachInterrupt(digitalPinToInterrupt(M1eA), M1counter, RISING);
  attachInterrupt(digitalPinToInterrupt(M2eA), M2counter, RISING);
  attachInterrupt(digitalPinToInterrupt(M3eA), M3counter, RISING);

  pinMode(3, OUTPUT);
  pinMode(M1eA, INPUT_PULLUP);
  pinMode(M1eB, INPUT_PULLUP);
  pinMode(M2eA, INPUT_PULLUP);
  pinMode(M2eB, INPUT_PULLUP);
  pinMode(M3eA, INPUT_PULLUP);
  pinMode(M3eB, INPUT_PULLUP);
  interrupts();
  //feed forward
  long setpwm[5] = { 0, 1023, 2046, 3069, 4092 };
  float m1rpmlist[5], m2rpmlist[5], m3rpmlist[5];
  for (int i = 0; i < 5; i++) {
    motormove(M1IN1, M1IN2, setpwm[i], true);
    motormove(M2IN1, M2IN2, setpwm[i], true);
    motormove(M3IN1, M3IN2, setpwm[i], true);
    delay(500);
    readrpm();
    m1rpmlist[i] = m1rpm;
    m2rpmlist[i] = m2rpm;
    m3rpmlist[i] = m3rpm;
  }
  float m1xy_sum = 0, m2xy_sum = 0, m3xy_sum = 0;
  float m1x_sum = 0;
  float m1sqr_sum = 0;
  float m1y_sum = 0, m2y_sum = 0, m3y_sum = 0;
  for (int i = 0; i < 5; i++) {
    m1xy_sum += m1rpmlist[i] * setpwm[i];
    m2xy_sum += m2rpmlist[i] * setpwm[i];
    m3xy_sum += m3rpmlist[i] * setpwm[i];
    m1x_sum += setpwm[i];
    m1sqr_sum += setpwm[i] * setpwm[i];
    m1y_sum += m1rpmlist[i];
    m2y_sum += m2rpmlist[i];
    m3y_sum += m3rpmlist[i];
  }

  m1coefficient = (5 * m1xy_sum - m1x_sum * m1y_sum) / (5 * m1sqr_sum - m1x_sum * m1x_sum);  //calculate coefficient
  m2coefficient = (5 * m2xy_sum - m1x_sum * m2y_sum) / (5 * m1sqr_sum - m1x_sum * m1x_sum);
  m3coefficient = (5 * m3xy_sum - m1x_sum * m3y_sum) / (5 * m1sqr_sum - m1x_sum * m1x_sum);
  m1section = (m1y_sum - m1coefficient * m1x_sum) / 5.0f;  //calculate section
  m2section = (m2y_sum - m2coefficient * m1x_sum) / 5.0f;
  m3section = (m3y_sum - m3coefficient * m1x_sum) / 5.0f;

  //sound
  kasuko();
  //start
  before_time = millis();
}

void loop() {
  if (BTSerial.available()) {
    //receive
    BTSerial.readBytesUntil('\n', buf, 5);
    for (int i = 0; i < 5; i++) {
      parity_tmp += __builtin_popcount(buf[i]);
    }
    if (parity_tmp % 2 == 1) return;

    //read
    //stick
    xbuf = (buf[0] << 8) | buf[1];
    if (xbuf > 1023 || xbuf < 0) return;
    ybuf = (buf[2] << 8) | buf[3];
    if (ybuf > 1023 || ybuf < 0) return;
    //switch
    switch_tmp = buf[4];
    /* 
      0:choozing servo1,1:choozing servo2
      2:way of moving
      3:lever1,4:lever2
    */
    for (int i = 0; i < 5; i++) {
      switch_tmp = switch_tmp >> 1;
      if ((switch_tmp) % 2 == 0) switch_buf[i] = true;
      else switch_buf[i] = false;
    }
    //data check
    if (switch_buf[0] && switch_buf[1]) return;
    else if (switch_buf[3] && switch_buf[4]) return;
    //operation
    //arm
    //chooze arm
    if (switch_buf[0]) arm_switch = 0;
    else if (switch_buf[1]) arm_switch = 2;
    else arm_switch = 1;

    //move arm
    if (switch_buf[3] || switch_buf[4]) {
      if (switch_buf[4]) {
        arm_direction = true;
      } else if (switch_buf[3]) {
        arm_direction = false;
      }

      if (arm_switch == 0) {
        motormove(M1IN3, M1IN4, arm_speed, arm_direction);
      } else if (arm_switch == 1) {
        motormove(M2IN3, M2IN4, arm_speed, arm_direction);
      } else {
        motormove(M3IN3, M3IN4, arm_speed, arm_direction);
      }
    }
    //foot
    if (switch_buf[2] == 0) {
      if (xbuf < 502 || xbuf > 532 || ybuf < 502 || ybuf > 532) {
        //calculation
        xbuf = map(xbuf, 0, 1023, -511, 511);
        ybuf = map(ybuf, 0, 1023, -511, 511);

        cood_dig = atan2(ybuf, xbuf);
        if (cood_dig < 0) cood_dig += 2 * PI;
        speed = map(sqrt((long)xbuf * xbuf + (long)ybuf * ybuf), 0, 723, 0, 1023);
        w1 = -1 * speed * cos(cood_dig + PI);
        w2 = -1 * speed * cos(cood_dig + 5 * PI / 3);
        w3 = -1 * speed * cos(cood_dig + PI / 3);

        if (w1 >= 0) w1dir = true;
        else w1dir = false;
        if (w2 >= 0) w2dir = true;
        else w2dir = false;
        if (w3 >= 0) w3dir = true;
        else w3dir = false;

        w1 = map(abs(w1), 0, 1023, 0, 4095);
        w2 = map(abs(w2), 0, 1023, 0, 4095);
        w3 = map(abs(w3), 0, 1023, 0, 4095);

        float m1error = m1_target_rpm - m1rpm;  //rpm error
        float m2error = m2_target_rpm - m2rpm;
        float m3error = m3_target_rpm - m3rpm;

        error_now_time = micros();
        float dt = ((float)error_now_time - (float)error_before_time) / 1000000.0;
        m1Kd = (m1error - before_m1error) / dt / m1coefficient * Kd_tune;  //set defferencial PWM
        m2Kd = (m2error - before_m2error) / dt / m2coefficient * Kd_tune;  //control noize to tuning Kd_tune bias
        m3Kd = (m3error - before_m3error) / dt / m3coefficient * Kd_tune;

        error_before_time = error_now_time;  //error_time_duration between before error

        m1Kp = m1error / m1coefficient * Kp_tune;  // change the pwm error
        m2Kp = m2error / m2coefficient * Kp_tune;
        m3Kp = m3error / m3coefficient * Kp_tune;

        w1 = w1 + m1Kp + m1Kd;  //give bias value to w1
        w2 = w2 + m2Kp + m2Kd;  //control noize to tuning Kp_tune bias
        w3 = w3 + m3Kp + m3Kd;

        w1 = constrain(w1, 0, 4095);  //constaining
        w2 = constrain(w2, 0, 4095);
        w3 = constrain(w3, 0, 4095);

        m1_target_rpm = m1coefficient * w1 + m1section;
        m2_target_rpm = m2coefficient * w2 + m2section;
        m3_target_rpm = m3coefficient * w3 + m3section;

        before_m1error = m1error;  //make error log to make Kd_gain
        before_m2error = m2error;
        before_m3error = m3error;
        //move
        motormove(M1IN1, M1IN2, w1, w1dir);
        motormove(M2IN1, M2IN2, w2, w2dir);
        motormove(M3IN1, M3IN2, w3, w3dir);
      } else {
        w1 = 0;
        w2 = 0;
        w3 = 0;
        motormove(M1IN1, M1IN2, w1, true);
        motormove(M2IN1, M2IN2, w2, true);
        motormove(M3IN1, M3IN2, w3, true);
      }
    } else {
      m1Kp = 0;
      m2Kp = 0;
      m3Kp = 0;
      m1Kd = 0;
      m2Kd = 0;
      m3Kd = 0;
      if (xbuf < 502 || xbuf > 532) {
        //calculation
        xbuf = map(xbuf, 0, 1023, -511, 511);
        //move
        if (xbuf >= 0) {
          w1 = map(xbuf, 0, 511, 0, 4095);
          w2 = map(xbuf, 0, 511, 0, 4095);
          w3 = map(xbuf, 0, 511, 0, 4095);
          motormove(M1IN1, M1IN2, w1, true);
          motormove(M2IN1, M2IN2, w2, true);
          motormove(M3IN1, M3IN2, w3, true);
        } else {
          w1 = map(abs(xbuf), 0, 511, 0, 4095);
          w2 = map(abs(xbuf), 0, 511, 0, 4095);
          w3 = map(abs(xbuf), 0, 511, 0, 4095);
          motormove(M1IN1, M1IN2, w1, false);
          motormove(M2IN1, M2IN2, w2, false);
          motormove(M3IN1, M3IN2, w3, false);
        }
      } else {
        w1 = 0;
        w2 = 0;
        w3 = 0;
        motormove(M1IN1, M1IN2, w1, true);
        motormove(M2IN1, M2IN2, w2, true);
        motormove(M3IN1, M3IN2, w3, true);
      }
    }
  }
  //initialization
  parity_tmp = 0;
  //lighting
  if (lighting_flag) {
    if (LED_pwm < 100) {
      if (LEDpin_location[LED_count]) analogWrite(LEDpins[LED_count], 0);
      else pwm.setPWM(LEDpins[LED_count], 0, 0);
      if (LED_count == 7) LED_count = 0;
      else LED_count++;
      LED_pwm = 4095;
      if (LEDpin_location[LED_count]) analogWrite(LEDpins[LED_count], map(LED_pwm, 0, 4095, 0, 1023));
      else pwm.setPWM(LEDpins[LED_count], 0, LED_pwm);
    } else {
      LED_pwm -= 100;
      if (LEDpin_location[LED_count]) analogWrite(LEDpins[LED_count], map(LED_pwm, 0, 4095, 0, 1023));
      else pwm.setPWM(LEDpins[LED_count], 0, LED_pwm);
    }
    lighting_flag = false;
  }
  //read rpm
  current_time = millis();
  if (current_time - before_time > encoder_read_late) readrpm();
}
