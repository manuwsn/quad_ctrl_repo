#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <malloc.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "quad_ctrl_node.hpp"

#define DEBUG 1
int action=0;

class Quad_Ctrl_Node
{
public:
  Quad_Ctrl_Node();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
  void vertScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
  void b_update_cache(int x);
  void push(int y, int w);
  stack_type * pop();
  int all_ones(int lx,int ly,int ux,int uy);
  void grow_ones(int lx,int ly, int* ux, int* uy);
  ros::NodeHandle nh_;

  ros::Publisher cmd_pub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber vert_laser_sub_;

  geometry_msgs::Twist twist;

  int time;
  char b[BEAM*RANGES];
  int cache[RANGES];
  stack_type * stack;

};

Quad_Ctrl_Node::Quad_Ctrl_Node():
  time(0)
{
 
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/quad_cmd_twist", 1);
  
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/vrep/front_scan", 10, &Quad_Ctrl_Node::scanCallback, this);
  vert_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/vrep/vert_front_scan", 10, &Quad_Ctrl_Node::vertScanCallback, this);

  stack = NULL;
}

int area(int lx, int ly, int rx, int ry){
  if (lx > rx || ly > ry)
    return 0;
  else
    return (rx-lx+1)*(ry-ly+1);
}


int Quad_Ctrl_Node::all_ones(int lx,int ly,int ux,int uy){
  int i, j;
  for(i=lx;i<=ux;i++)
    for(j=ly;j<=uy;j++)
      if(b[i*RANGES+j]==0)
	return 0;
  return 1;
}

void Quad_Ctrl_Node::vertScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
  {
 
    float bottom = laserScan->ranges[0];
    float top =  laserScan->ranges[1];
    float z=0;
    float halfHight = (bottom+top)/2;
    if(bottom>halfHight*1.2)
      z=-0.2;
    else if(bottom<halfHight*0.8)
      z=+0.2;
    else
      z=0;
    twist.linear.z = z;
    if(DEBUG)
    printf("x : %f, y :  %f, z : %f, xA : %f\n",twist.linear.x, twist.linear.y, twist.linear.z,twist.angular.z);
    
    cmd_pub_.publish(twist);
  }

void Quad_Ctrl_Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan)
  {
    int ib=0,ir;
    for(ir=0;ir<BEAM;ir++){
      float range = laserScan->ranges[ir];
      if (range==0) range=MAX_BIM;
      int br = range*RANGES/MAX_BIM;
      memset(&b[ib],1,br);
      memset(&b[ib+br],0,RANGES-br+1);
      ib+=RANGES;
    }

  
    intervall t_int[BEAM];
    int i_y,i_x;
    int found=0,i_int=0,distance=0,wide=0;
    int b_lx=0,b_ry=0;
    
    while(!found){
      for(i_y=RANGES-1;i_y>=0&&!found;i_y--){
	int state=0;
	i_int=0;
	memset(t_int,0,BEAM*sizeof(intervall));
	for(i_x=0;i_x<BEAM;i_x++){
	  int pt =  b[i_x*RANGES+i_y];
	  switch(state){
	  case 0:
	    switch(pt){
	    case 1:
	      state=1;
	      t_int[i_int].lx=i_x;
	      t_int[i_int].ry=i_x;
	      break;
	    };
	    break;
	  case 1:
	    switch(pt){
	    case 0:
	      state=0;
	      i_int++;
	      break;
	    case 1:
	      t_int[i_int].ry=i_x;
	      if(i_x==BEAM-1)
		i_int++;
	      break;
	    };
	    break;
	  }
	}
	int i_check,max=0;
	for(i_check=0;i_check<i_int;i_check++){
	  if(t_int[i_check].ry-t_int[i_check].lx+1>max){
	    b_lx=t_int[i_check].lx;
	    b_ry=t_int[i_check].ry;
	    max=t_int[i_check].ry-t_int[i_check].lx+1;
	  }
	}
	if(max >= 5){
	  found=1;
	  distance=i_y;
	  wide=max;
	}
      }
      if(!found)
	found=1;
    }  //while (!found)

    int irect=b_lx+(b_ry-b_lx)/2;
    
    float x,vmax=5;
    if(distance!=0)
      x=vmax*(distance+1)/RANGES*wide/BEAM;
    else
      x=0;

    int i_lr;
    float left=0,right=0;
    for(i_lr=0;i_lr<SAFE_SIDE;i_lr++){
      left+=laserScan->ranges[i_lr];
      right+=laserScan->ranges[BEAM-i_lr-1];
    }
    float y=0;
    float halfLarge = (left+right)/2;
    if(left>halfLarge*1.2)
      y=-0.2;
    else if (left<halfLarge*0.8)
      y=0.2;
    else y=0;
    
    
    if(DEBUG){
      int cb=0;
      for(cb=1;cb <= RANGES;cb++){
	for(ib=RANGES-cb;ib<BEAM*RANGES;ib+=RANGES)
	  if(ib/RANGES== b_lx || ib/RANGES == b_ry || RANGES-cb==distance)
	    printf("\033[47;01m%i\033[00m",b[ib]);
	  else if(ib/RANGES == irect)
	    printf("\033[20;01m%i\033[00m",b[ib]);
	  else
	    printf("%i",b[ib]);
	printf("\n");
      }
      printf("\n\n");
    }
    
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = ((irect)*3.14)/BEAM - (3.14/2);
}
    



int main(int argc, char** argv)
{
  ros::init(argc, argv, "quad_scan_control");
  Quad_Ctrl_Node quad_scan_control;

  ros::spin();

    


}
