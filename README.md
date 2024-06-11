# -2-

#include <Wire.h>
#include <LSM303.h>
#include <NewPing.h>

LSM303 compass;

float heading = 0.0;
float target_heading_angle = 0.0;

#define MAX_DISTANCE 150       // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE 210  // mm 단위
#define WALL_GAP_DISTANCE_HALF 210  // mm 단위
#define MOTOR_PWM_OFFSET 10

#define Front 0
#define Left  1 
#define Right 2

#define FTRIGGER_PIN 13
#define FECHO_PIN 12
#define LTRIGGER_PIN 16
#define LECHO_PIN 17
#define RTRIGGER_PIN 14
#define RECHO_PIN 15

NewPing sonar[3] = {   
    NewPing(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE), // Front sensor
    NewPing(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE), // Left sensor
    NewPing(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE)  // Right sensor
};

#define ENR 6
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 8
#define ENL 7

float front_sonar = 0.0;
float left_sonar = 0.0;
float right_sonar = 0.0;

int maze_status = 0; 
bool flag[6] = {1,1,1,1,1,1};

void setup() 
{
  Wire.begin();
  compass.init();
  compass.enableDefault();
  flag[0] = 0;

  pinMode(FTRIGGER_PIN, OUTPUT);
  pinMode(FECHO_PIN, INPUT);

  pinMode(LTRIGGER_PIN, OUTPUT);
  pinMode(LECHO_PIN, INPUT);

  pinMode(RTRIGGER_PIN, OUTPUT);
  pinMode(RECHO_PIN, INPUT);
  
  pinMode(ENR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENL, OUTPUT);
  Serial.begin(115200); // 통신속도를 115200으로 설정
}

void motor_A_control(int direction_a, int motor_speed_a) // 모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENR, motor_speed_a);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENR, motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) // 모터 B의 방향(direction)과 속도(speed) 제어
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENL, motor_speed_b);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENL, motor_speed_b);
  }
}

void imu_rotation_right(void)
{
  if(target_heading_angle >= 360)
  {
    while(1)
    {
      compass.read();
      heading = compass.heading();
      if(heading < 180)
      {
        heading += 360;
      }
      motor_A_control(HIGH, 250);
      motor_B_control(LOW, 250);
      if(heading >= target_heading_angle)
      {
        break;
      }
    }
  }
  else
  {
    while(heading < target_heading_angle)
    {
      compass.read();
      heading = compass.heading();
      motor_A_control(HIGH, 250);
      motor_B_control(LOW, 250);
      if(heading >= target_heading_angle)
      {
        break;
      }
    }
  }
}

void imu_rotation_left(void)
{
  if(target_heading_angle <= 0)
  {
    target_heading_angle += 360;
    while(1)
    {
      compass.read();
      heading = compass.heading();
      if(heading > 180)
      {
        heading -= 360;
      }
      motor_A_control(LOW, 250);
      motor_B_control(HIGH, 250);
      if(heading <= target_heading_angle)
      {
        break;
      }
    }
  }
  else
  {
    while(heading > target_heading_angle)
    {
      compass.read();
      heading = compass.heading();
      heading += 360;
      motor_A_control(LOW, 250);
      motor_B_control(HIGH, 250);
      if(heading <= target_heading_angle)
      {
        break;
      }
    }
  }
}

void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 100; // Value to be adjusted later (how much to turn)
  int right_pwm = 0;
  int left_pwm = 0;
  error = (right_sonar - left_sonar); // Change here to prioritize avoiding obstacles on the left
  error = Kp * error;

  if(error >= 50) error = 50;
  if(error <= -50) error = -50;

  right_pwm = base_speed - error; // Change here to prioritize avoiding obstacles on the left
  left_pwm = base_speed + error;

  if(left_pwm <= 0) left_pwm = 0;
  if(right_pwm <= 0) right_pwm = 0;

  if(left_pwm >= 255) left_pwm = 255; // Increase maximum speed
  if(right_pwm >= 255) right_pwm = 255; // Increase maximum speed

  motor_A_control(HIGH, left_pwm); // Left forward
  motor_B_control(HIGH, right_pwm); // Right forward
}

void loop() 
{
  front_sonar = sonar[Front].ping_cm() * 10;
  left_sonar = sonar[Left].ping_cm() * 10;
  right_sonar = sonar[Right].ping_cm() * 10;

  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10;
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE * 10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;

  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  if(flag[0] == 0)
  {
    wall_collision_avoid(215);
    if(left_sonar >= WALL_GAP_DISTANCE)
    {
      motor_A_control(HIGH, 150);
      motor_B_control(HIGH, 255);

      compass.read();
      heading = compass.heading();
      target_heading_angle = heading - 90;
      imu_rotation_left();

      flag[0] = 1;
      flag[1] = 0;
    }
  }
  else if(flag[1] == 0)
  {
    wall_collision_avoid(215);
    if(front_sonar <= WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      target_heading_angle = heading + 90;  
      imu_rotation_right(); // 우회전

      flag[1] = 1;
      flag[2] = 0;
    }
  }
  else if(flag[2] == 0)
  {
    wall_collision_avoid(215);
    if(front_sonar <= WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      target_heading_angle = heading + 180;    
      imu_rotation_right(); // 유턴

      flag[2] = 1;
      flag[3] = 0;
    }
  }
  else if(flag[3] == 0)
  {
    wall_collision_avoid(215);
    if(front_sonar <= WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      target_heading_angle = heading - 90;
      imu_rotation_left(); // 좌회전

      flag[3] = 1;
      flag[4] = 0;
    }
  }
  else if(flag[4] == 0)
  {
    wall_collision_avoid(215);
    if(front_sonar <= WALL_GAP_DISTANCE_HALF)
    {
      compass.read();
      heading = compass.heading();
      target_heading_angle = heading - 90;
      imu_rotation_left(); // 좌회전

      flag[4] = 1;
      flag[5] = 0;
    }
  }
  else if(flag[5] == 0)
  {
    wall_collision_avoid(215);
  }
}
