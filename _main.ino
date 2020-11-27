#include"zigbee.h"
#include"mp.h"
#include"state.h"
#include"ctrl.h"

//DEBUG
#define print(x) Serial.println(x)

#define ObstacleNumber 8//有10条障碍物的线
int spd = 51;
String stest = "4848484848484848484848";
State* gst; //游戏状态
_mp* mp;

//int PackagePos[10][2];
//barrier[6][6][4]=0;//标记道路可通行性
 

void TURN_TO(int x){
    int dt=gst->cdir-x;
    if(dt<0)dt+=4;
    if(dt == 3)turn_left();
    else if(dt == 2)turn_left(),turn_left();
    else if(dt == 1)turn_right();
    gst->cdir=x;
}

int ACT(int x,int y,int dir=-1){
  mp -> min_dis2(gst->cx,gst->cy,x,y,1,gst);
  gst->cx = x, gst->cy = y;
  //cout<<x<<" "<<y<<endl;
  for(int i=1;i<=mp->don;i++){
      if(mp->DO[i]<=6){
          go_n_steps(mp->DO[i]);
      }else if(mp->DO[i]==7){
          turn_left();
      }else{
          turn_right();
      }
  }
  if(dir!=-1)TURN_TO(dir);
}
/*int go(int XX=6){
  for(int i=0;i<8;i++)bt[1<<i]=i;

  ans=1000;
  X[0]=cx,Y[0]=cy,pre[0]=0;
  for(int i=0;i<=6;i++)
    for(int j=0;j<=6;j++)
      dis[i][j]=min_dis2(X[i],Y[i],X[j],Y[j],0);
  _go(1,(1<<XX)-1,0);
  for(int i=1;i<=XX;i++)ACT(X[pans[i]],Y[pans[i]]);
  //cout<<ans<<endl;
  //ACT(X[8],Y[8]);
}*/

void stage1(){
    Serial.println("..");
    go_one_step(2);
    digitalWrite(48,HIGH);
    delay(1000);
    digitalWrite(48,LOW);
    //ACT(1,5);
    TURN_TO(0);
    go_one_step(2);
}

void stage2(){
  go_one_step(1);
}

void setup()
{ 
    gst = new State();
    mp = new _mp();
    //Serial2.begin(115200);
    track_pinint(); //循迹引脚初始化
    Serial.begin(9600);
    digitalWrite(48,LOW);
    stage2();
}

void loop()
{
    
}


/*void stage2(){
  go_one_step(2);
  //get_task();
  //Serial.println(",,");
  for(int i=0;i<8;i++)bt[1<<i]=i;
  int hs0 = -1, hs, type = 0;
  while(1){
    hs = get_hs();
    if(hs != hs0){
      X[0]=cx,Y[0]=cy,pre[0]=0;
      visa=(1<<6)-1;
      for(int i=0;i<6;i++){
        X[i+1]=PackagePos[i][0];
        Y[i+1]=PackagePos[i][1];  
            //cout<<X[i+1]<<" "<<Y[i+1]<<endl;
      }//cout<<visa<<",,";
      for(int i=0;i<=6;i++)
          for(int j=0;j<=6;j++)
              dis[i][j]=min_dis2(X[i],Y[i],X[j],Y[j],0);
     }
     if(!type)
       X[7]=getPassengerstartposY(),Y[7]=getPassengerstartposX();
     else
       X[7]=getPassengerfinalposY(),Y[7]=getPassengerfinalposX();

     trans(X[7],Y[7]);
      
     int cnt = 1;
     for(int i=visa,j;i;i-=(i&-i)){
       j=bt[i&-i]+1; cnt++;
       dis[j][7]=dis[7][j]=min_dis2(X[j],X[j],X[7],Y[7],0);
     }
     dis[0][7]=dis[7][0]=min_dis2(X[0],X[0],X[7],Y[7],0);
    
     ans=1000; 
     _go(1,visa|(1<<6),0,0);
   //Serial.println(",,,,,");
    //for(int i=1;i<=7;i++)cout<<X[i]<<" "<<Y[i]<<endl;
    //for(int i=1;i<=7;i++)cout<<pans[i]<<" "; 
    //for(int i=2;i<=7;i++)cout<<dis[pans[i]][pans[i-1]]<<endl;
    //cout<<"...................\n";
     for(int i=1;i<=cnt;i++){
          if(pans[i]==7){
            ACT(X[7],Y[7]);
              break;
          }
          int t=pans[i];
          ACT(X[t],Y[t]);
          visa-=(1<<(t-1));
      }
      //cout<<ans;
      type ^= 1;
     //return;
  }
}*/
/*
void stage2(){
    
   //while(1){
     //   receive_data();
       // getpackage();
         //get_pass();
          //Serial.println(getPackageposX(0));
       // delay(10);
     
    go_one_step(2);
    //Serial.println(",");
    get_hs();
    for(int i=0;i<6;i++){
        X[i+1]=PackagePos[i][0];
        Y[i+1]=PackagePos[i][1];
     }

    
    int pd=get_hs(),p2;
        
    go();

    while(1){
        p2=get_hs();
        receive_data();
        getpackage();
        Serial.println(p2);
        Serial.println(pd);
        if(p2>20&&p2!=pd)break;
    }
    Serial.println(p2);
    Serial.println(pd);
    
     digitalWrite(48,HIGH);
    
     get_hs();
     for(int i=0;i<6;i++){
        X[i+1]=PackagePos[i][0];
        Y[i+1]=PackagePos[i][1];
     }
     go();
}*/




