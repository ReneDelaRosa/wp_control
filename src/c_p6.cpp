/*
	Prueba 6
	Detección de waypoints
*/

#include <ros/ros.h>
#include <unistd.h>
#include <math.h>
#include <cmath> 
#include <vector>
#include <iostream>
#include <termios.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

//Variables
int intOp,cx_r=0,cy_r=0,r=320,k=0,state=1;//r=230
uint8_t battery;
float vx=0.0,vy=0.0,vaz=0.0,vy_odom=0.0;
bool fang0=false;
vector<float> angs{{0.0, 330.0}};
vector<float> angs_conv(angs.size());
geometry_msgs::Twist emma_cmd;
std_msgs::Float32 ang_des;


//Funciones
void c_vel (float lin_x, float lin_y, float lin_z, float ang_z);
float conv_ang (float ang_d);

//Callbacks
void cy_callback(const std_msgs::Int32::ConstPtr& msg){
	cy_r=msg->data;
}

void opt_callback(const std_msgs::Int32::ConstPtr& msg1){
	intOp=msg1->data;
}

void battery_callback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg2){
	battery=msg2->percent;
}

void controly_callback(const std_msgs::Float32::ConstPtr& msg3){
	vy=msg3->data;
}

void controlaz_callback(const std_msgs::Float32::ConstPtr& msg4){
	vaz=msg4->data;
}

void controlyodom_callback(const std_msgs::Float32::ConstPtr& msg5){
	vy_odom=msg5->data;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "MCN");
	ros::NodeHandle nodo_;

	//Publishers y subscribers

	//Despegue
	ros::Publisher takeoff_pub_ = nodo_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	//Aterrizaje
	ros::Publisher land_pub_ = nodo_.advertise<std_msgs::Empty>("/bebop/land", 1);
	//Velocidad
	ros::Publisher fb_pub = nodo_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);//bebop/cmd_vel
	//Angulo deseado para control en z
	ros::Publisher ang_pub=nodo_.advertise<std_msgs::Float32>("/angle",10);
	//Teclado	
	ros::Subscriber tec_sub = nodo_.subscribe("/copt",10,opt_callback);
	//Bateria
	ros::Subscriber battery_ = nodo_.subscribe("/bebop/states/common/CommonState/BatteryStateChanged",1,battery_callback);
	//Centroide en x de la imagen (y del dron)
	ros::Subscriber cy_sub = nodo_.subscribe("/c_y",10,cy_callback);
	//Control en y con momentos
	ros::Subscriber controly_ = nodo_.subscribe("/controlY",1,controly_callback);
	//Control en az
	ros::Subscriber controlaz_ = nodo_.subscribe("/velaz",1,controlaz_callback);
	//Control en y con odometria
	ros::Subscriber controlyodom_ = nodo_.subscribe("/vely",1,controlyodom_callback);

	std_msgs::Empty takeoff_cmd;
  	std_msgs::Empty land_cmd;

  	//Obtencion de los valores de angulos para el nodo de control
  	for (int i=0;i<angs.size();i++){
		angs_conv[i]=conv_ang(angs[i]);
	}	

	while(ros::ok()){
		switch(intOp){
			case 49:
				switch(state){
					case 1://Primer giro para ir hacia la primera marca.
						cout<<"Prueba 6 C1 "<<"B = "<<(int)battery<<" % "<<"Primer giro "<<k<<" vaz= "<< vaz<<" M= "<<k<< endl;
						ang_des.data=angs_conv[0];
						ang_pub.publish(ang_des);
						c_vel(0.0,0.0,0.0,vaz);
						fb_pub.publish(emma_cmd);
						ros::Duration(2).sleep();
						state=2;
					break;
					case 2:
						if(cy_r==-1){//El dron avanza en linea recta cuando no detecta ninguna marca.
							cout<<"Prueba 6 C2 "<<"B = "<<(int)battery<<" % "<<"No detectado "<<" vaz= "<< vaz<<" M= "<<k<< endl;
							c_vel(0.5,vy_odom,0.0,vaz);//vaz, vy_odom 0.05
							fb_pub.publish(emma_cmd);
							state=2;
						}
						else{
							state=3;
						}	

					break;
					case 3://Control de posición cuando la marca es detectada.
						if (cy_r<r&&cy_r>=0){//El centroide aun no llega al punto especificado.
							cout<<"Prueba 6 C3 "<<"B = "<<(int)battery<<" % "<<"Avanzando "<<"vy: "<< vy<<" vaz= "<< vaz<<" M= "<<k<< endl;
							c_vel(0.7,vy,0.0,vaz);//vaz
							fb_pub.publish(emma_cmd);
							state=3;
						}
						else{//El centroide llegó al punto especificado,
							state=4;
						}

					break;
					case 4://Hover después de la deteccion.
						std::cout<<"Prueba 6 C4 "<<"B = "<<(int)battery<<" % "<<"Hover wp"<<" vaz= "<< vaz<<" M= "<<k<< endl;
						c_vel(0.0,0.0,0.0,vaz);
						fb_pub.publish(emma_cmd);
						ros::Duration(4).sleep();
						state=5;
					break;
					case 5://Giro despues del hover.
						k++;
						if(k==angs.size()){//Completados todos los giros ve al caso 7 de aterrizaje.
							state=7;															
						}
						else{//Realiza un hover de 2 segundos.
							cout<<"Prueba 6 C5 "<<"B = "<<(int)battery<<" % "<<"Giro wp"<<k<<" vaz= "<< vaz<<" M= "<<k<< endl;
							ang_des.data=angs_conv[k];
							ang_pub.publish(ang_des);
							c_vel(0.0,0.0,0.0,vaz);
							fb_pub.publish(emma_cmd);
							ros::Duration(2).sleep();
							state=6;	
						}
					break;
					case 6://Si después del giro sigue detectando algo: avanza hasta que deje de detectarlo.
						if(cy_r!=-1){//&& cy_r>=300
							cout<<"Prueba 6 C6 "<<"B = "<<(int)battery<<" % "<<"Avanzando "<<" vaz= "<< vaz<<" M= "<<k<< endl;
							c_vel(0.5,vy_odom,0.0,vaz);//vaz, vy_odom
							fb_pub.publish(emma_cmd);
							state=6;
						}
						else{
							state=2;
						}
					break;
					case 7://Aterrizaje despues de recorrer todas las marcas.
						cout<<"Prueba 6 C7 "<<"B = "<<(int)battery<<" % "<<"Aterrizando :) "<<" vaz= "<< vaz<<" M= "<<k<< endl;
						k=0;
						c_vel(0.0,0.0,0.0,vaz);//vaz
						fb_pub.publish(emma_cmd);
						ros::Duration(1).sleep();
						land_pub_.publish(land_cmd);
						state=1;
					break;
				}				
				break;
				case 50:
					std::cout<<"Take Off "<<"B "<<(int)battery<<" % \n";
					takeoff_pub_.publish(takeoff_cmd);
				break;
				case 51:
					std::cout<<"Land "<<"B "<<(int)battery<<" % \n";
					land_pub_.publish(land_cmd);
				break;
				case 52:
					std::cout<<"Hover "<<"B "<<(int)battery<<" % \n";
					c_vel(0.0,0.0,0.0,0.0);
					fb_pub.publish(emma_cmd);
				break;
				default:	
					std::cout<<"Hover "<<"B "<<(int)battery<<" % \n";
					c_vel(0.0,0.0,0.0,0.0);
					fb_pub.publish(emma_cmd);
				break;
		}
		ros::spinOnce();
	}
	return 0;
}

void c_vel (float lin_x, float lin_y, float lin_z, float ang_z){
	emma_cmd.linear.x = lin_x;
  	emma_cmd.linear.y = lin_y;
  	emma_cmd.linear.z = lin_z;
  	emma_cmd.angular.z = ang_z;
}

float conv_ang(float ang_d){
	double cn1=-2.981e-10,cn2=2.223e-07,cn3=-1.798e-05,cn4=-0.007787,cn5=0.004498;
	double cp1=5.749e-10 ,cp2=-5.023e-07,cp3=0.000128,cp4=-0.01078,cp5=1.119;
	float p;
	//Si el angulo es negativo
	if (ang_d<0.0){
		ang_d=360-abs(ang_d);
	}
	//Curve fitting
	if (ang_d>=0&&ang_d<180){//Valores de 0 a 179.9
		p=cn1*pow(ang_d,4)+cn2*pow(ang_d,3)+cn3*pow(ang_d,2)+cn4*ang_d+cn5;
	}
	else if(ang_d>=180&&ang_d<=360){//Valores de 180 a 360
		p=cp1*pow(ang_d,4)+cp2*pow(ang_d,3)+cp3*pow(ang_d,2)+cp4*ang_d+cp5;
		cout<<p<<endl;	
	}
	return p;
}

//ang_des.data=conv_ang(45);
//ang_pub.publish(msg);
