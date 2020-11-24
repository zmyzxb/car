#include "zigbee.h"
class state{
public:
  int PackagePos[8][2];
  int barrier[8][8][2];

  void trans(int &a,int &b) {
    if(42<=a&&a<=62){a=1;}
    else if(72<=a&&a<=92){a=2;}
    else if(102<=a&&a<=122){a=3;}
    else if(132<=a&&a<=152){a=4;}
    else if(162<=a&&a<=182){a=5;}
    else if(192<=a&&a<=212){a=6;}
    else a=-10;
    if(42<=b&&b<=62){b=1;}
    else if(72<=b&&b<=92){b=2;}
    else if(102<=b&&b<=122){b=3;}
    else if(132<=b&&b<=152){b=4;}
    else if(162<=b&&b<=182){b=5;}
    else if(192<=b&&b<=212){b=6;}
    else b=-10;
  }
  void getpackage(){
    int a,b;
    for(int i=0;i<6;i++){
      b=getPackageposX(i),
      a=getPackageposY(i);
      trans(a,b);
      PackagePos[i][0]=a;
      PackagePos[i][1]=b;
    }
  }
  void get_pass(){
    int A,B,t,x1,y1,x2,y2;
    for(int i=0;i<8;i++){
      x1=getObstacleAposY(i);
      y1=getObstacleAposX(i);
      x2=getObstacleBposY(i);
      y2=getObstacleBposX(i);
      
      if(x1-x2<5&&x1-x2>-5){
        A=y1/30;//42 102
        B=y2/30;
        int X=x1/30;
        if(A>B)t=A,A=B,B=t;
        for(int j=A;j<B;j++){
            barrier[X][j][0]=1;
            barrier[X-1][j][2]=1;
        } 
      }
      if(y1-y2<5&&y1-y2>-5){
        A=x1/30;
        B=x2/30;
        if(A>B)t=A,A=B,B=t;
        int Y=y1/30;
        for(int j=A;j<B;j++){
            barrier[j][Y][1]=1;
            barrier[j][Y-1][3]=1;
        } 
      }
    }
  }
};