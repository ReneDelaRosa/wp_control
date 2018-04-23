#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
//Variables
//ARDrone m=1.477
//Bebop m=0.450
//Bebop2 m=0.500
float  m = 0.45, g = 9.81;
float psi, dpsi, wpsi, psid = 0.0, psig = 0.0, dpsig = 0.0, wpsig = 0.0, psi0=0.0, dpsi0 = 0.0, wpsi0 = 0.0, psig0 = 0.0, dpsig0 = 0.0, wpsig0 = 0.0, upsi=0.0, vaz=0.0;
using namespace std;
//Funciones
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	psi=msg->pose.pose.orientation.z;
}
//Edicion 1.0
void angle_callback(const std_msgs::Float32::ConstPtr& angle_message){
	psid=angle_message->data;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "c_vaz");
	ros::NodeHandle node_obj;
	//Publisher
	ros::Publisher vaz_publisher=node_obj.advertise<std_msgs::Float32>("/velaz",10);
	//Suscriber
	ros::Subscriber odom = node_obj.subscribe("/bebop/odom",10,odom_callback);
	//Edicion 1.0
	ros::Subscriber angle_received = node_obj.subscribe("/angle",10,angle_callback);
	ros::Rate loop_rate(10);
	std::cout<<"Velocidad Az \n";
	while (ros::ok()){
		//Inicio control de orientación
		std::cout<<"Angulo deseado: "<<psid<<"\n";
		upsi = (psid-psi)-1.4*(psi-psi0)-1.0*wpsig;
		if(upsi>0.75){	
			upsi=0.75;
		}
		if(upsi<-0.75){
			upsi=-0.75;
		}
		//Observador
 		psig  = psig0 + 0.001*dpsig0 + pow(0.001,2)*upsi/(2.0*m) + tanh(psi0-psig0);
		dpsig = dpsig0 + 0.001*upsi/m + tanh(psi0-psig0);
		wpsi  = psi0-psig0;
		wpsig = wpsig0 + tanh(wpsi0-wpsig0);
		//Fin de control de orientación
		cout << "VelaZ" << vaz << "\n";
		vaz=upsi;
		std_msgs::Float32 msg;
		msg.data=vaz;
		vaz_publisher.publish(msg);
		//Reinicio de Variables
		psi0 = psi; psig0 = psig; wpsi0 = wpsi; wpsig0 = wpsig;
		ros::spinOnce();
	}	
	return 0;	
}
