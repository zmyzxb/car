 /********************************
zigbee.h
接受上位机的数据
接收说明（以USART2为例）
  在程序开始的时候使用zigbee_Init(&huart2)进行初始化;
  在回调函数中使用zigbeeMessageRecord(void)记录数据，并重新开启中断

数据说明
    struct BasicInfo Game;储存比赛状态、时间、泄洪口信息
    struct CarInfo CarInfo;//储存车辆信息
    struct PassengerInfo Passenger;//储存人员的信息、位置和送达位置
    struct PackageInfo Package[6];//储存物资的信息
    struct StopInfo Stop[2];//储存隔离点位置信息
    struct ObstacleInfo Obstacle[8];//储存虚拟障碍物信息
    通过接口获取数据
**********************************/
#ifndef ZIGBEE_H
#define ZIGBEE_H
//#include "stm32f1xx_hal.h"
#define INVALID_ARG -1
#define ZIGBEE_MESSAGE_LENTH 70

struct Position
{
    unsigned int X;
    unsigned int Y;
};

struct BasicInfo
{
    uint8_t GameState;  //游戏状态：00未开始，01进行中，10暂停，11结束
    uint16_t Time;  //比赛时间，以0.1s为单位
    uint8_t stop;   //泄洪口开启信息
};
struct CarInfo
{
    struct Position pos;    //小车位置
    uint16_t score;         //得分
    uint8_t picknum;         //小车成功收集物资个数
    uint8_t task;           //小车任务，0上半场，1下半场
    uint8_t transport;         //小车上是否有人
    uint8_t transportnum;      //小车运送人的个数
    uint8_t area;    //小车所在的区域
    uint8_t WhetherRightPos;//小车这次位置信息是否是正确的，1是正确的，0是不正确的.
};
struct PassengerInfo
{
    struct Position startpos;   //人员初始位置
    struct Position finalpos;   //人员要到达的位置
};
struct PackageInfo
{
    uint8_t No;               //物资编号
    struct Position pos;
    uint8_t whetherpicked;    //物资是否已被拾取
};
struct FloodInfo
{
    uint8_t FloodNo;
    struct Position pos;
};
struct ObstacleInfo
{
    uint8_t ObstacleNo;
    struct Position posA;
    struct Position posB;
};
enum GameStateEnum
{
  GameNotStart, //未开始
  GameGoing,    //进行中
  GamePause,    //暂停中
  GameOver      //已结束
};
/**************接口*************************/
//void zigbee_Init(UART_HandleTypeDef *huart);//初始化
//void zigbeeMessageRecord(void);             //实时记录信息，在每次接收完成后更新数据，重新开启中断

int getGameState(void);      //比赛状态
uint16_t getGameTime(void);                   //比赛时间，单位为0.1s
uint16_t getPassengerstartposX(void);     //人员初始位置
uint16_t getPassengerstartposY(void);
struct Position getPassengerstartpos(void);
uint16_t getPassengerfinalposX(void);           //人员需到达位置
uint16_t getPassengerfinalposY(void);
struct Position getPassengerfinalpos(void);
uint16_t getGameFlood(void);               //隔离点开启信息
uint16_t getFloodposX(int FloodNo);     //隔离点位置X
uint16_t getFloodposY(int FloodNo);           //隔离点位置Y
struct Position getFloodpos(int FloodNo);     //隔离点位置
uint16_t getCarposX();        //小车x坐标
uint16_t getCarposY();      //小车y坐标
struct Position getCarpos();  //小车位置
//uint16_t getCarWhetherRightPos();//小车这次位置信息是否是正确的
uint16_t getPackageposX(int PackNo);        //物资x坐标
uint16_t getPackageposY(int PackNo);      //物资y坐标
uint16_t getPackagewhetherpicked(int PackNo);   //物资是否已被收集
struct Position getPackagepos(int PackNo);  //物资位置
uint16_t getCarpicknum();//小车收集数
uint16_t getCartransportnum();//小车运送人员数
uint16_t getCartransport();//小车是否正在运送人员
uint16_t getCarscore();//小车得分
uint16_t getCartask();//小车任务
uint16_t getCararea();//小车区域
uint16_t getObstacleAposX(int ObstacleNo);        //虚拟障碍Ax坐标
uint16_t getObstacleAposY(int ObstacleNo);      //虚拟障碍Ay坐标
uint16_t getObstacleBposX(int ObstacleNo);          //虚拟障碍Bx坐标
uint16_t getObstacleBposY(int ObstacleNo);          //虚拟障碍By坐标
struct Position getObstacleApos(int ObstacleNo);  //虚拟障碍A位置
struct Position getObstacleBpos(int ObstacleNo);  //虚拟障碍B位置
void Decode();
int receiveIndexMinus(int index_h, int num);
int receiveIndexAdd(int index_h, int num);
//#endif // V0_5_H_INCLUDED


volatile uint8_t zigbeeReceive[ZIGBEE_MESSAGE_LENTH];  //实时记录收到的信息
volatile uint8_t zigbeeMessage[ZIGBEE_MESSAGE_LENTH];//经过整理顺序后得到的信息
volatile int message_index = 0;
volatile int message_head = -1;
uint8_t zigbeeBuffer[1];

//UART_HandleTypeDef* zigbee_huart;


volatile struct BasicInfo Game;//储存比赛状态、时间、泄洪口信息
volatile struct CarInfo Car;//储存车辆信息
volatile struct PassengerInfo Passenger;//储存人员的信息、位置和送达位置
volatile struct PackageInfo Package[6];//储存防汛物资的信息
volatile struct FloodInfo Flood[5];//储存泄洪口位置信息
volatile struct ObstacleInfo Obstacle[8];//储存虚拟障碍信息
/***********************接口****************************/
/*void zigbee_Init(UART_HandleTypeDef *huart)
{
  zigbee_huart = huart;
  HAL_UART_Receive_IT(zigbee_huart, zigbeeBuffer, 1);
}

void zigbeeMessageRecord(void)
{
  zigbeeMessage[message_index] = zigbeeBuffer[0];
  message_index = receiveIndexAdd(message_index, 1);    //一个简单的索引数增加函数

  if (zigbeeMessage[receiveIndexMinus(message_index, 2)] == 0x0D
    && zigbeeMessage[receiveIndexMinus(message_index, 1)] == 0x0A)//一串信息的结尾
  {
    if (receiveIndexMinus(message_index, message_head) == 0)
    {

      int index = message_head;
      for (int i = 0; i < 70; i++)
      {
        zigbeeReceive[i] = zigbeeMessage[index];
        index = receiveIndexAdd(index, 1);
      }
      Decode();
    }
    else
    {
      message_head = message_index;
    }
  }
  HAL_UART_Receive_IT(zigbee_huart, zigbeeBuffer, 1);
}*/



int getGameState(void)
{
  uint8_t state = Game.GameState;

  if (state == 0)
  {
    return 0;
  }
  else if (state == 1)
  {
    return 1;
  }
  else if (state == 2)
  {
    return 2;
  }
  else if (state == 3)
  {
    return 3;
  }

  //return GameNotStart;
}
uint16_t getGameTime(void)
{
  return Game.Time;
}
uint16_t getGameFlood(void)
{
    return (uint16_t)Game.stop;
}
uint16_t getPassengerstartposX(void)
{
    return Passenger.startpos.X;
}
uint16_t getPassengerstartposY(void)
{
    return Passenger.startpos.Y;
}
/*struct Position getPassengerstartpos(void)
{
    return Passenger.startpos;
}*/
uint16_t getPassengerfinalposX(void)
{
    return Passenger.finalpos.X;
}
uint16_t getPassengerfinalposY(void)
{
    return Passenger.finalpos.Y;
}
/*struct Position getPassengerfinalpos(void)
{
    return Passenger.finalpos;
}*/
uint16_t getFloodposX(int FloodNo)
{
    return Flood[FloodNo].pos.X;
}
uint16_t getFloodposY(int FloodNo)
{
    return Flood[FloodNo].pos.Y;
}
/*struct Position getFloodpos(int FloodNo)
{
    return Flood[FloodNo].pos;
}*/
uint16_t getCarposX()
{
    return (uint16_t)Car.pos.X;

}
uint16_t getCarposY()
{
    return (uint16_t)Car.pos.Y;
}
/*struct Position getCarpos()
{
    return Car.pos;
}*/
uint16_t getCarWhetherRightPos()
{
    return (uint16_t)Car.WhetherRightPos;
}

uint16_t getPackageposX(int PackNo)
{
    if (PackNo != 0 && PackNo != 1 && PackNo != 2 && PackNo != 3 && PackNo != 4 && PackNo != 5)
    return (uint16_t)INVALID_ARG;
  else
    return (uint16_t)Package[PackNo].pos.X;
}
uint16_t getPackageposY(int PackNo)
{
    if (PackNo != 0 && PackNo != 1 && PackNo != 2 && PackNo != 3 && PackNo != 4 && PackNo != 5)
    return (uint16_t)INVALID_ARG;
  else
    return (uint16_t)Package[PackNo].pos.Y;
}
uint16_t getPackagewhetherpicked(int PackNo)
{
  if (PackNo != 0 && PackNo != 1 && PackNo != 2 && PackNo != 3 && PackNo != 4 && PackNo != 5)
    return (uint16_t)INVALID_ARG;
  else
    return (uint16_t)Package[PackNo].whetherpicked;
}
/*struct Position getPackagepos(int PackNo)
{
    return Package[PackNo].pos;
}*/
uint16_t getCarpicknum()
{
    return (uint16_t)Car.picknum;
}
uint16_t getCartransportnum()
{
    return (uint16_t)Car.transportnum;
}
uint16_t getCartransport()
{
    return (uint16_t)Car.transport;
}
uint16_t getCarscore()
{
    return (uint16_t)Car.score;
}
uint16_t getCartask()
{
    return (uint16_t)Car.task;
}
uint16_t getCararea()
{
    return (uint16_t)Car.area;
}
uint16_t getObstacleAposX(int ObstacleNo)       //虚拟障碍Ax坐标
{ 
    return (uint16_t)Obstacle[ObstacleNo].posA.X;
}
uint16_t getObstacleAposY(int ObstacleNo)       //虚拟障碍Ax坐标
{
    return (uint16_t)Obstacle[ObstacleNo].posA.Y;
}
uint16_t getObstacleBposX(int ObstacleNo)       //虚拟障碍Ax坐标
{
    return (uint16_t)Obstacle[ObstacleNo].posB.X;
}
uint16_t getObstacleBposY(int ObstacleNo)     //虚拟障碍Ax坐标
{
    return (uint16_t)Obstacle[ObstacleNo].posB.Y;
}
/*struct Position getObstacleApos(int ObstacleNo)
{
    return Obstacle[ObstacleNo].posA;
}
struct Position getObstacleBpos(int ObstacleNo)
{
    return Obstacle[ObstacleNo].posB;
}*/
/***************************************************/

void DecodeBasicInfo()
{
  Game.Time = (zigbeeReceive[0] << 8) + zigbeeReceive[1];
  Game.GameState = (zigbeeReceive[2] & 0xC0) >> 6;
  Game.stop=(zigbeeReceive[2]& 0x07);
}
void DecodeCarInfo()
{
    Car.pos.X=(zigbeeReceive[3]);
    Car.pos.Y=(zigbeeReceive[4]);
    Car.score=(zigbeeReceive[32]<<8)+zigbeeReceive[33];
    Car.picknum=zigbeeReceive[35];
    Car.task=((zigbeeReceive[2] & 0x20)>>5);
    Car.transport=((zigbeeReceive[2] & 0x08)>>3);
    Car.transportnum=(zigbeeReceive[34]);
    Car.area=((zigbeeReceive[19] & 0x02)>>1);
    Car.WhetherRightPos = (zigbeeReceive[19] & 0x01);
}

void DecodePassengerInfo()
{
    Passenger.startpos.X=(zigbeeReceive[15]);
    Passenger.startpos.Y=(zigbeeReceive[16]);
    Passenger.finalpos.X=(zigbeeReceive[17]);
    Passenger.finalpos.Y=(zigbeeReceive[18]);
}
void DecodePackageAInfo()
{
    Package[0].pos.X=(zigbeeReceive[20]);
    Package[0].pos.Y=(zigbeeReceive[21]);
    Package[0].whetherpicked=((zigbeeReceive[19] & 0x80)>>7);
}
void DecodePackageBInfo()
{
    Package[1].pos.X=(zigbeeReceive[22]);
    Package[1].pos.Y=(zigbeeReceive[23]);
    Package[1].whetherpicked=((zigbeeReceive[19] & 0x40)>>6);
}
void DecodePackageCInfo()
{
    Package[2].pos.X=(zigbeeReceive[24]);
    Package[2].pos.Y=(zigbeeReceive[25]);
    Package[2].whetherpicked=((zigbeeReceive[19] & 0x20)>>5);
}
void DecodePackageDInfo()
{
    Package[3].pos.X=(zigbeeReceive[26]);
    Package[3].pos.Y=(zigbeeReceive[27]);
    Package[3].whetherpicked=((zigbeeReceive[19] & 0x10)>>4);
}
void DecodePackageEInfo()
{
    Package[4].pos.X=(zigbeeReceive[28]);
    Package[4].pos.Y=(zigbeeReceive[29]);
    Package[4].whetherpicked=((zigbeeReceive[19] & 0x08)>>3);
}
void DecodePackageFInfo()
{
    Package[5].pos.X=(zigbeeReceive[30]);
    Package[5].pos.Y=(zigbeeReceive[31]);
    Package[5].whetherpicked=((zigbeeReceive[19] & 0x04)>>2);
}
void DecodeFloodInfo()
{
    Flood[0].pos.X=(zigbeeReceive[5]);
    Flood[0].pos.Y=(zigbeeReceive[6]);
    Flood[1].pos.X=(zigbeeReceive[7]);
    Flood[1].pos.Y=(zigbeeReceive[8]);
    Flood[2].pos.X=(zigbeeReceive[9]);
    Flood[2].pos.Y=(zigbeeReceive[10]);
    Flood[3].pos.X=(zigbeeReceive[11]);
    Flood[3].pos.Y=(zigbeeReceive[12]);
    Flood[4].pos.X=(zigbeeReceive[13]);
    Flood[4].pos.Y=(zigbeeReceive[14]);
}
void DecodeObstacle()
{
    int i;
    for(i=0;i<8;i++)
    {
        Obstacle[i].posA.X=(zigbeeReceive[36+i*4]);
        Obstacle[i].posA.Y=(zigbeeReceive[37+i*4]);
        Obstacle[i].posB.X=(zigbeeReceive[38+i*4]);
        Obstacle[i].posB.Y=(zigbeeReceive[39+i*4]);
    }


}
void Decode()
{
  DecodeBasicInfo();
  DecodeCarInfo();
  DecodePassengerInfo();
  DecodePackageAInfo();
  DecodePackageBInfo();
  DecodePackageCInfo();
  DecodePackageDInfo();
  DecodePackageEInfo();
  DecodePackageFInfo();
  DecodeFloodInfo();
  DecodeObstacle();
}
int receiveIndexMinus(int index_h, int num)
{
  if (index_h - num >= 0)
  {
    return index_h - num;
  }
  else
  {
    return index_h - num + ZIGBEE_MESSAGE_LENTH;
  }
}

int receiveIndexAdd(int index_h, int num)
{
  if (index_h + num < ZIGBEE_MESSAGE_LENTH)
  {
    return index_h + num;
  }
  else
  {
    return index_h + num - ZIGBEE_MESSAGE_LENTH;
  }
}

void receive_data(){
  while (Serial2.available()){
    zigbeeMessage[message_index] = Serial2.read();
    message_index = receiveIndexAdd(message_index, 1);    //一个简单的索引数增加函数

    if (zigbeeMessage[receiveIndexMinus(message_index, 2)] == 0x0D
      && zigbeeMessage[receiveIndexMinus(message_index, 1)] == 0x0A){
      if (receiveIndexMinus(message_index, message_head) == 0){
        int index = message_head;
        for (int i = 0; i < 70; i++)
        {
          zigbeeReceive[i] = zigbeeMessage[index];
          index = receiveIndexAdd(index, 1);
        }
        Decode();
      }
      else
      {
        message_head = message_index;
      }
    }
  }
}

#endif
