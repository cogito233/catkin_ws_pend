#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include<bits/stdc++.h>
#define ll long long
#define int long long
#define dt 1e-6
#define double long double
struct vec{
	double x,y;
}vec_zero,vec_G;
inline vec operator+(const vec &x,const vec &y){
	vec z;z.x=x.x+y.x;z.y=x.y+y.y;
	return z;
}inline vec operator-(const vec &x,const vec &y){
	vec z;z.x=x.x-y.x;z.y=x.y-y.y;
	return z;
}inline vec operator*(const vec &x,const double &y){//向量数乘 
	vec z;z.x=x.x*y;z.y=x.y*y;
	return z;
}inline vec operator/(const vec &x,const double &y){
	vec z;z.x=x.x/y;z.y=x.y/y;
	return z;
}inline double operator*(const vec &x,const vec &y){//点积 
	return x.x*y.x+x.y*y.y;
}inline double mo(vec x){return sqrt(x.x*x.x+x.y*x.y);}
struct ball{//点的位置，速度方向，当前在圆弧上的位置，速度大小 
	vec position,velocity_direct;
	double radian,velocity;
}x;
signed main(signed argc, char **argv){
	ros::init(argc, argv, "pend");

	ros::NodeHandle n;

	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Float32>("pend_x", 1000);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float32>("pend_y", 1000);

	ros::Rate loop_rate(100);

	double length=10;//摆长
	int t=0;
	x.position.x=length;x.velocity_direct.y=1;
	vec_G.y=9.8;//重力 
	double c1=0,c2=0;//流体力学的阻尼修正参数，目前设为0
	while(ros::ok()){//1e-5秒为一个单位，每0.01s输出一次位置 
		if (t%10000==0){
			std_msgs::Float32 msgx,msgy;
			msgx.data = (float)x.position.x;
			msgy.data = (float)x.position.y;
			ROS_INFO("%f %f", msgx.data,msgy.data);
			chatter_pub1.publish(msgx);
			chatter_pub2.publish(msgy);
			ros::spinOnce();
			loop_rate.sleep();
			//printf("%.5Lf %.5Lf \n",x.position.x,x.position.y);
		}
		++t; 
		//根据当前位置计算加速度 
		double dv=x.velocity_direct*vec_G-c2*x.velocity;
		//在一小段圆弧上近似的认为delta_v是相等的 
		if (x.velocity<0)dv+=c1*x.velocity*x.velocity;
			else dv-=c1*x.velocity*x.velocity;
		//阻尼修正
		double dx=(x.velocity*2+dv*dt)*dt/2;
		//在一小段时间内运动的距离 
		x.radian+=dx/length;
		x.position.x=cos(x.radian)*length;x.position.y=-sin(x.radian)*length;
		x.velocity_direct.x=-sin(x.radian);x.velocity_direct.y=cos(x.radian);
		//算出新的位置和速度方向 
		x.velocity+=dv*dt;
	}
} 
