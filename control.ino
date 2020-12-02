// #include "TimerOne.h"
#define A_order 0
#define B_order 1
#define C_order 2
#define D_order 3
#define D1_Pin 34
#define D2_Pin 23
#define D3_Pin 35
#define D4_Pin 25
#define D5_Pin 36
#define D6_Pin 27
#define D7_Pin 37
#define D8_Pin 29
#define D9_Pin 38
#define D10_Pin 30
#define D11_Pin 39
#define D12_Pin 31
#define D13_Pin 40
#define D14_Pin 32
#define D15_Pin 42
#define D16_Pin 33
#define DL_Pin 22
#define DR_Pin 24
bool rush = 1;
bool isOnLine = 0;
int alreadyGo = 0;
String stest = "33" /*7777777777777777888888888888888*/;
int encoder_Pin[4][2] //[i][0]判断方向,[i][1]计数
    = {53, 3, 52, 18, 51, 2, 50, 19};
float kspd = 1;
const uint8_t PWM_PIN[4][2] = {7, 8, 11, 12, 6, 5, 44, 46}; //[7,8]左前轮(7>8前转) [11,12]右前，[6,5]左后，[44,46]右后
float Kp = 25, Ki = 0, Kd = 10;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0, previous_error = 0, previous_I = 0;
int sdspd, sdspd1 = 100, sdspd2 = 180, sdspd3 = 200, sdspd4, sdspd5, sdspd6, linespd = 180, turnspd = 200;
int D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16, DL, DR;
unsigned long csgap[2] = {150, 25};
void pinint()
{
    pinMode(48, OUTPUT);
    pinMode(D1_Pin, INPUT);
    pinMode(D2_Pin, INPUT);
    pinMode(D3_Pin, INPUT);
    pinMode(D4_Pin, INPUT);
    pinMode(D5_Pin, INPUT);
    pinMode(D6_Pin, INPUT);
    pinMode(D7_Pin, INPUT);
    pinMode(D8_Pin, INPUT);
    pinMode(D9_Pin, INPUT);
    pinMode(D10_Pin, INPUT);
    pinMode(D11_Pin, INPUT);
    pinMode(D12_Pin, INPUT);
    pinMode(D13_Pin, INPUT);
    pinMode(D14_Pin, INPUT);
    pinMode(D15_Pin, INPUT);
    pinMode(D16_Pin, INPUT);
    pinMode(DL_Pin, INPUT);
    pinMode(DR_Pin, INPUT);
    pinMode(encoder_Pin[A_order][0], INPUT);
    pinMode(encoder_Pin[A_order][1], INPUT);
}
void read_sensor_values()
{
    // digitalWrite(48,HIGH);
    D1 = digitalRead(D1_Pin);
    D2 = digitalRead(D2_Pin);
    D3 = digitalRead(D3_Pin);
    D4 = digitalRead(D4_Pin);
    D5 = digitalRead(D5_Pin);
    D6 = digitalRead(D6_Pin);
    D7 = digitalRead(D7_Pin);
    D8 = digitalRead(D8_Pin);
    D9 = digitalRead(D9_Pin);
    D10 = digitalRead(D10_Pin);
    D11 = digitalRead(D11_Pin);
    D12 = digitalRead(D12_Pin);
    D13 = digitalRead(D13_Pin);
    D14 = digitalRead(D14_Pin);
    D15 = digitalRead(D15_Pin);
    D16 = digitalRead(D16_Pin);
    DL = digitalRead(DL_Pin);
    DR = digitalRead(DR_Pin);
//     if(D16) {digitalWrite(48,HIGH);delay(500);digitalWrite(48,LOW);}
    if (D16 && !isOnLine)
    {
        alreadyGo++;
        isOnLine ^= 1;
        // digitalWrite(48,HIGH);
    }
    else if (!D16 && isOnLine)
    {
        isOnLine ^= 1;
        // digitalWrite(48,LOW);
    }
}
void motorsWrite(int speedL, int speedR)
{
    if (speedR > 0)
    {
        analogWrite(11, speedR);
        analogWrite(12, 0);
        analogWrite(44, speedR);
        analogWrite(46, 0);
    }
    else
    {
        analogWrite(11, 0);
        analogWrite(12, -speedR);
        analogWrite(44, 0);
        analogWrite(46, -speedR);
    }

    if (speedL > 0)
    {
        analogWrite(7, speedL);
        analogWrite(8, 0);
        analogWrite(6, speedL);
        analogWrite(5, 0);
    }
    else
    {
        analogWrite(7, 0);
        analogWrite(8, -speedL);
        analogWrite(6, 0);
        analogWrite(5, -speedL);
    }
}
void go_straight()
{
    int left_motor_speed = linespd + PID_value;
    int right_motor_speed = linespd - PID_value;

    if (left_motor_speed < -255)
    {
        left_motor_speed = -255;
    }
    if (left_motor_speed > 255)
    {
        left_motor_speed = 255;
    }
    if (right_motor_speed > 255)
    {
        right_motor_speed = 255;
    }
    if (right_motor_speed < -255)
    {
        right_motor_speed = -255;
    }
    motorsWrite(left_motor_speed * kspd, right_motor_speed * kspd);
}
void turn_left()
{

    while (!digitalRead(D7_Pin))
    {
        motorsWrite(-turnspd * kspd, turnspd * kspd);
    }
    while (digitalRead(D9_Pin) || digitalRead(D10_Pin) || digitalRead(D11_Pin))
    {
        motorsWrite(turnspd * kspd, -turnspd * kspd);
    }
    pull_off();
}
void turn_right()
{

    while (!digitalRead(D9_Pin))
    {
        motorsWrite(turnspd * kspd, -turnspd * kspd);
    }
    while (digitalRead(D7_Pin) || digitalRead(D6_Pin) || digitalRead(D5_Pin))
    {
        motorsWrite(-turnspd * kspd, turnspd * kspd);
    }
    pull_off();
}
void pull_off()
{
    for (int i = 0; i < 4; i++) //设置电机引脚
    {
        analogWrite(PWM_PIN[i][0], 0);
        analogWrite(PWM_PIN[i][1], 0);
    }
}
void shut_down(int n)
{
    if (n == 1)
        sdspd = sdspd1;
    else if (n == 2)
        sdspd = sdspd2;
    else if (n == 3)
        sdspd = sdspd3;
    else if (n == 4)
        sdspd = sdspd4;
    else if (n == 5)
        sdspd = sdspd5;
    else
        sdspd = sdspd6;
    while (rush)
        motorsWrite(-sdspd * kspd, -sdspd * kspd);
    pull_off();
}
void calc_pid()
{
    P = error;
    I = I + error;
    D = error - previous_error;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    previous_error = error;
}
void go_n_steps(int n)
{
    // int m = 0;
    //unsigned long time1 = millis();
    // bool cs_find = 0;
    // read_sensor_values();
    //bool edge = 0;
    while (true)
    {
        // int flag = 1;
        // for (int i = 0; i < 15; i++)
        // {
        // delay(2);
        read_sensor_values();
        //if(millis()-time1>200&&(D16||D1))
        //   edge=1;
        /*if ((millis()-time1>200) 
                    && ((D6&&D7)|| 
                        (D7&&D8)||
                        (D8&&D9)||
                        (D9&&D10)||
                        (D10&&D11||
                        (D11&&D12))) && edge)*/
        // if (!D1 && !D16)
        // cs_find = 1;
        if (alreadyGo == n)
        // if (DL || DR)
        {
            // m++;
            // cs_find = 0;
            // edge=0;
            // time1 = millis();
            // if (m == n)
            // {
            shut_down(n);
            alreadyGo = 0;
            // flag = 0;
            break;
            // }
            // delay(cstime);
        }
        else if (D8 && D9) //8和9通道在线上
        {
            error = 0;
        }
        else if (D8 && !D9) //8通道在线上
        {
            error = -2;
        }
        else if (D9 && !D8) //9通道在线上
        {
            error = 2;
        }
        else if (D7) //7通道在线上
        {
            error = -3;
        }
        else if (D10) //10通道在线上
        {
            error = 3;
        }
        else if (D6) //6通道在线上
        {
            error = -4;
        }
        else if (D11) //11通道在线上
        {
            error = 4;
        }
        else if (D5) //5通道在线上
        {
            error = -5;
        }
        else if (D12) //12通道在线上
        {
            error = 5;
        }
        // else if (D4) //4通道在线上
        // {
        //     error = -5;
        // }
        // else if (D13) //13通道在线上
        // {
        //     error = 5;
        // }
        // else if (D3) //3通道在线上
        // {
        //     error = -6;
        // }
        // else if (D14) //14通道在线上
        // {
        //     error = 6;
        // }
        // else if (D2) //2通道在线上
        // {
        //     error = -7;
        // }
        // else if (D15) //15通道在线上
        // {
        //     error = 7;
        // }
        // else if (D1) //1通道在线上
        // {
        //     error = -8;
        // }
        // else if (D16) //16通道在线上
        // {
        //     error = 8;
        // }
        else
            error = error * 0.9;
        calc_pid();
        go_straight();
    }
    // if (!flag)
    // break;
}
void string_to_act(String s)
{
    for (int i = 0; i < s.length(); i++)
    {
        if (s[i] == '7')
            turn_left();
        else if (s[i] == '8')
            turn_right();
        else
            go_n_steps(s[i] - '0');
        delay(10);
    }
}
void isRush()
{

    if (digitalRead(encoder_Pin[A_order][1]) == LOW)
    { //如果是下降沿触发的中断
        // Serial.println(digitalRead(encoder_Pin[A_order][0]));
        // Serial.println(digitalRead(encoder_Pin[A_order][1]));
        // Serial.println('x');
        if (digitalRead(encoder_Pin[A_order][0]) == HIGH)
        { //根据另外一相电平判定方向

            rush = 0;
        }
        else
        {
            rush = 1;
        }
    }
    else
    { //如果是下降沿触发的中断
        // Serial.println(digitalRead(encoder_Pin[A_order][0]));
        // Serial.println(digitalRead(encoder_Pin[A_order][1]));
        // Serial.println('x');
        if (digitalRead(encoder_Pin[A_order][0]) == LOW)
        { //根据另外一相电平判定方向

            rush = 0;
        }
        else
        {
            rush = 1;
        }
    }
}
void setup()
{
    // Serial.begin(9600);
    // Timer0.initialize(1000);
    // Timer0.attachInterrupt(read_sensor_values);
    attachInterrupt(1, isRush, CHANGE);
    pinint();
    string_to_act(stest);
}
void loop()
{
//  read_sensor_values();
}
