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
#define left 1
#define down 2
#define right 3
#define up 0 

float Kt6 = 2, Kt5 = 1.5, Kt4 = 1.7, Kt3 = 1.8, Kt2 = 1.7, Kt1 = 1, Kt; //刹车时间系数
float cs_delay_time = 170;
float Ksn = 0.35, Ks4=0.32 ,Ks1 = 0.38, Ks;                           //电机转速
const uint8_t PWM_PIN[4][2] = {7, 8, 11, 12, 6, 5, 44, 46}; //[7,8]左前轮(7>8前转) [11,12]右前，[6,5]左后，[44,46]右后
float Kp = 50, Ki = 1, Kd = 600;                            //pid弯道参数参数
int initial_motor_speed = 250;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0; //pid直道参数

float decide = 0;                         //元素判断
float previous_error = 0, previous_I = 0; //误差值
void read_sensor_values(void);            //读取初值
void calc_pid(void);                      //计算pid

int D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16, DL, DR;

void string_to_act(String s);
void pre_shut_down(int spd);
void go_straight_r();
void shutdown(int x);

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
void track_pinint()
{
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
    pinMode(48,OUTPUT);
}

void turn_left()
{
    while (!digitalRead(D1_Pin))
    {
        motorsWrite(-75, 75);
    }
    while (!digitalRead(D6_Pin))
    {
        motorsWrite(-35, 35);
    }
    if (digitalRead(D9_Pin))
    {
        motorsWrite(75, -75);
        delay(100);
    }
    pull_off();
}
void turn_right()
{
    while (!digitalRead(D16_Pin))
    {
        motorsWrite(75, -75);
    }
    while (!digitalRead(D11_Pin))
    {
        motorsWrite(35, -35);
    }
    if (digitalRead(D8_Pin))
    {
        motorsWrite(-75, 75);
        delay(100);
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

void go_straight()
{
    int left_motor_speed = initial_motor_speed + PID_value;
    int right_motor_speed = initial_motor_speed - PID_value;

    if (left_motor_speed < -255)
    {
        left_motor_speed = -255;
    }
    if (left_motor_speed > 255)
    {
        left_motor_speed = 255;
    }
    motorsWrite(left_motor_speed * Ks, right_motor_speed * Ks);
}
void go_straight_r()
{
    int left_motor_speed = initial_motor_speed + PID_value;
    int right_motor_speed = initial_motor_speed - PID_value;

    if (left_motor_speed < -255)
    {
        left_motor_speed = -255;
    }
    if (left_motor_speed > 255)
    {
        left_motor_speed = 255;
    }
    motorsWrite(-left_motor_speed * Ks, -right_motor_speed * Ks);
}
void calc_pid()
{
    P = error;
    I = I + error;
    D = error - previous_error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_error = error;
}
void shut_down(int x)
{
    switch (x)
    {
    case 6:
        Kt = Kt6;
        break;
    case 5:
        Kt = Kt5;
        break;
    case 4:
        Kt = Kt4;
        break;
    case 3:
        Kt = Kt3;
        break;
    case 2:
        Kt = Kt2;
        break;
    case 1:
        Kt = Kt1;
        break;
    default:
        break;
    }
    analogWrite(11, 0);
    analogWrite(12, 150);
    analogWrite(44, 0);
    analogWrite(46, 150);
    analogWrite(7, 0);
    analogWrite(8, 150);
    analogWrite(6, 0);
    analogWrite(5, 150);
    delay(100 * Kt);
    pull_off();
}
void read_sensor_values()
{
    //    D1 = digitalRead(D1_Pin);
    //    D2 = digitalRead(D2_Pin);
    //    D3 = digitalRead(D3_Pin);
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
    //    D14 = digitalRead(D14_Pin);
    //    D15 = digitalRead(D15_Pin);
    //    D16 = digitalRead(D16_Pin);
    DL = digitalRead(DL_Pin);
    DR = digitalRead(DR_Pin);
}
void go_n_steps(int m)
{
  //motorsWrite(100,100);
  //delay(100);
  //pull_off();
    if(m>4){
        go_n_steps(3);
        go_n_steps(m-3);
        return;
    }
    int n = m;
    read_sensor_values();
    while (!D1 && !D2 && !D3 && !D4 && !D5 && !D6 && !D7 && !D8 && !D9 && !D10 && !D11 && !D12 && !D13 && !D14 && !D15 && !D16)
    {
        motorsWrite(50, 50);
        read_sensor_values();
    }
    Ks = Ksn;
    if(m==4) Ks=Ks4;
    while (n > 1)
    {
        for (int k = 0; k < 20; k++)
        {
            delay(2);
            read_sensor_values();
            if ((D8 && D11) || (D9 && D6)) //十字路
            {
                n--;
                motorsWrite(250 * Ksn, 250 * Ksn);
                delay(cs_delay_time);
            }
            else if (D8 && D9) //8和9通道在线上
            {
                error = 0;
            }
            else if (D8 && !D9) //8通道在线上
            {
                error = -1;
            }
            else if (D9 && !D8) //9通道在线上
            {
                error = 1;
            }
            else if (D7) //7通道在线上
            {
                error = -2;
            }
            else if (D10) //10通道在线上
            {
                error = 2;
            }
            else if (D6) //6通道在线上
            {
                error = -3;
            }
            else if (D11) //11通道在线上
            {
                error = 3;
            }
            else if (D5) //5通道在线上
            {
                error = -4;
            }
            else if (D12) //12通道在线上
            {
                error = 4;
            }
            else if (D4) //4通道在线上
            {
                error = -5;
            }
            else if (D13) //13通道在线上
            {
                error = 5;
            }
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
        }
        calc_pid();
        go_straight();
    }
    go_one_step(m);
}

void go_one_step(int x)
{
    int flag = 0;
    Ks = Ks1;
    while (true)
    {
        for (int k = 0; k < 15; k++)
        {
            delay(2);
            read_sensor_values();
            if ((D8 && D11) || (D9 && D6)) //十字路
            {
                pull_off();
                while (!DL && !DR)
                {
                    read_sensor_values();
                    if (x == 1)
                        motorsWrite(40, 40);
                }
                shut_down(x);
                flag = 1;
                break;
            }
            else if (D8 && D9) //8和9通道在线上
            {
                error = 0;
            }
            else if (D8 && !D9) //8通道在线上
            {
                error = -1;
            }
            else if (D9 && !D8) //9通道在线上
            {
                error = 1;
            }
            else if (D7) //7通道在线上
            {
                error = -2;
            }
            else if (D10) //10通道在线上
            {
                error = 2;
            }
            else if (D6) //6通道在线上
            {
                error = -3;
            }
            else if (D11) //11通道在线上
            {
                error = 3;
            }
            else if (D5) //5通道在线上
            {
                error = -4;
            }
            else if (D12) //12通道在线上
            {
                error = 4;
            }
            else if (D4) //4通道在线上
            {
                error = -5;
            }
            else if (D13) //13通道在线上
            {
                error = 5;
            }
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
        }
        calc_pid();
        if (flag)
            break;
        go_straight();
    }
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
        //delay(100);
    }
}
