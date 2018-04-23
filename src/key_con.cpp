#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>

using namespace std;
int c = 1000;

char getch() {
	char buf =0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr (0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr (0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");
	return (buf);		
}	

int main (int argc, char **argv){
	ros::init(argc, argv, "CT");
	ros::NodeHandle node_obj;
	ros::Publisher number_publisher=node_obj.advertise<std_msgs::Int32>("/copt",10);
	ros::Rate loop_rate(10);
	std::cout<<"Control Teclado \n";
	while (ros::ok()){
		//std::cout<<"Control Teclado \n";
		c=getch();
		cout<< c << " ";
		switch (c){
			case 113:
				cout<<"Prueba 1"<<endl;
				break;
			case 55: 
				cout<<"Take Off" <<endl; 
				break;
			case 56:
				cout<<"Forward" <<endl;
				break;
			case 57:
				cout<<"Land" <<endl;
				break;
			case 52://4
				cout<<"Hover" <<endl;	
				break;
			case 53:
				cout<<"Backward" <<endl;	
				break;
			case 54:
				cout<<"Right" <<endl;
				break;
			case 49: //1
				cout<<"Control Automatico" <<endl;
				break;
			case 50://2 
				cout<<"Take Off" <<endl;
				break;
			case 51://3
				cout<<"Land" <<endl;
				break;
			case 48:
				cout<<"Emergency" <<endl;
				break;
			case 97:
				cout<<"Up" <<endl;
				break;
			case 115:
				cout<<"Down" <<endl;
				break;
			case 114:
				cout<<"R Prueba" <<endl;
				break;
				
			default:
				cout<<"HOVER" <<endl;
				break;
				
		}
		std_msgs::Int32 msg;
		msg.data=c;
		number_publisher.publish(msg);
		ros::spinOnce();
	}	
	return 0;	
}
