#include "state.h"

const int N = 6;
const int PT = N*N+5; 
const int PTX = N*N*4+20;
const int M = N*N*32;
 
const int TURN_LEFT = 7;
const int TURN_RIGHT = 8;

/*
          dir = 0
dir = 1              dir = 3
          dir = 2;
*/
int n = N;
char dx[4]={-1,0,1,0};
char dy[4]={0,-1,0,1};
bool Map[N+1][N+1];
bool barrier[N+1][N+1][4];
int DO[40],don;
//int X[10],Y[10];
//int que[PTX];

int qx[100],qy[100],dir[100],_p[8][8],PP[40];
//int dis[PTX];
//bool ava[PTX], vis[PTX];
int _tot;
int cdir,cx=6,cy=2;
int pre[10],ans,pans[10],bt[555];
int dis[10][10],dis1[10][10],dis2[10][10];bool visa;
int _px[8][8],_py[8][8],PX[50],PY[50];


inline int index(int x,int y){
    return ((x-1)*n+y-1)<<2;
}

int fd_p1(int x1,int y1,int x,int y){
	if(x!=x1||y!=y1)
		fd_p1(x1,y1,x-dx[_p[x][y]],y-dy[_p[x][y]]);
	PP[++_tot]=_p[x][y];
}

int fd_p2(int x1,int y1,int x,int y){
  if(x!=x1||y!=y1)
    fd_p2(x1,y1,_px[x][y],_py[x][y]);
  PX[++_tot]=x;//cout<<x<<" "<<y<<endl;
  PY[_tot]=y;
}

int min_dis1(int x1,int y1,int x2,int y2,int c=0){
  int hd = 1, tl = 0;
  tl++; 
  qx[tl]=x1; qy[tl]=y1; ;

  for(int i=1;i<=6;i++)
    for(int j=1;j<=6;j++)
      dis1[i][j]=-1;
  dis1[x1][y1]=0;
  while(1){
		int x=qx[hd],y=qy[hd];
		hd++;
		for(int i=0;i<4;i++){
      int nx=x+dx[i],ny=y+dy[i];
      if(nx>=1&&nx<=6&&ny>=1&&ny<=6&&dis1[nx][ny]==-1){
        if(!Map[nx][ny]&&!barrier[x][y][i]){
          dis1[nx][ny]=dis1[x][y]+1;
          tl++;
          qx[tl]=nx,qy[tl]=ny;
        }
      }
		}
	}
	return dis1[x2][y2];
}

int min_dis2(int x1,int y1,int x2,int y2,int c=0){
  int hd = 1, tl = 0;
  tl++; 
  qx[tl]=x1; qy[tl]=y1; 
  int dd[4];
  dd[0]=cdir;
  dd[1]=cdir+1;if(dd[1]>3)dd[1]=0;
  dd[2]=cdir-1;if(dd[2]<0)dd[2]=3;
  dd[3]=cdir+2;if(dd[3]>3)dd[3]-=4;
  
  for(int i=1;i<=6;i++)
    for(int j=1;j<=6;j++)
      dis2[i][j]=-1;
  dis2[x1][y1]=0;
  
  while(1){
    int x=qx[hd],y=qy[hd],d=dis2[x][y];
    //cout<<x<<","<<y<<endl; 
    hd++;
    if(x==x2&&y==y2){
      if(!c) break;
      _tot=0; don = 0;
      fd_p2(x1,y1,x2,y2);
      if(x1==x2&&y1==y2)break;
      for(int i=1;i<_tot;i++){
        int DX=PX[i+1]-PX[i],DY=PY[i+1]-PY[i],d,di;
        if(DX<0)d=0,di=-DX;
        if(DX>0)d=2,di=DX;
        if(DY<0)d=1,di=-DY;
        if(DY>0)d=3,di=DY;
        int dt=cdir-d;
        if(dt<0)dt+=4;
        if(dt == 3)DO[++don] = TURN_LEFT;
        else if(dt == 2) DO[++don] = TURN_LEFT, DO[++don] = TURN_LEFT;
        else if(dt == 1) DO[++don] = TURN_RIGHT;
        DO[++don]=di;
        cdir=d; 
      }
      break;
    }
    
    for(int k=0;k<4;k++){
      int nx=x,ny=y,i=dd[k];
            while(1){
              if(barrier[nx][ny][i])break;
              nx+=dx[i]; ny+=dy[i]; 
              if(nx<1||nx>6||ny<1||ny>6)break;
                if(dis2[nx][ny]==-1){
                  _px[nx][ny]=x;
                  _py[nx][ny]=y;
                  dis2[nx][ny]=d+1;
                  qx[++tl]=nx; qy[tl]=ny;
              } 
            }  
    }
  }
  return (dis2[x2][y2]==-1)?1000:dis2[x2][y2];
}


void _go(int x,int v,int d,int lst){
  if(d>=ans)return;
  if(!v){
    ans=d;
    for(int i=1;i<=7;i++)
      pans[i]=pre[i];
    return;
  }
  for(int i=v;i;i-=(i&-i)){
    pre[x]=bt[i&-i]+1;
    _go(x+1,v-(i&-i),d+dis[pre[x]][lst],pre[x]);
  }
}


/*int min_dis1(int x1,int y1,int x2,int y2,int c){
	int hd = 1, tl = 0;
	tl++; 
	qx[tl]=x1; qy[tl]=y1; dir[tl]=cdir;

	for(int i=1;i<=6;i++)
		for(int j=1;j<=6;j++)
			dis1[i][j]=-1;
	dis1[x1][y1]=0;
	while(1){
		int x=qx[hd],y=qy[hd],d=dir[hd];
		//cout<<x<<" "<<y<<" "<<d<<endl;
		hd++;
		if(x==x2&&y==y2){
			if(!c) break;
			_tot=0; don = 0;
			fd_p1(x1,y1,x2,y2);
			if(dis1[x2][y2]==0) break;	
			int k=0;
			for(int i=2,j=0;i<=_tot;i++){
    			if(cdir!=PP[i]){
    				if(k)DO[++don]=k;
					int dt=cdir-PP[i];
					if(dt<0)dt+=4;
					if(dt == 3)DO[++don] = TURN_LEFT;
					else if(dt == 2) DO[++don] = TURN_LEFT, DO[++don] = TURN_LEFT;
					else if(dt == 1) DO[++don] = TURN_RIGHT;
					cdir=PP[i];
					k=1;//cout<<cdir<<"."<<P[i]<<"."<<i<<endl;
				}else{
					k++;
				}
    		}
    		DO[++don]=k;
			break;
		}
		int nx=x+dx[d],ny=y+dy[d];
		if(nx>=1&&nx<=6&&ny>=1&&ny<=6&&dis1[nx][ny]==-1){
			if(!Map[nx][ny]&&!barrier[x][y][d]){
				dis1[nx][ny]=dis1[x][y]+1;
				tl++;
				qx[tl]=nx,qy[tl]=ny,dir[tl]=d;
				_p[nx][ny]=d;
			}
		}
		for(int i=0;i<4;i++){
			if(i!=d){
				int nx=x+dx[i],ny=y+dy[i];
				if(nx>=1&&nx<=6&&ny>=1&&ny<=6&&dis1[nx][ny]==-1){
					if(!Map[nx][ny]&&!barrier[x][y][i]){
						dis1[nx][ny]=dis1[x][y]+1;
						tl++;
						qx[tl]=nx,qy[tl]=ny,dir[tl]=i;
						_p[nx][ny]=i;
					}
				}
			}
		}
	}
	return dis1[x2][y2];
}*/