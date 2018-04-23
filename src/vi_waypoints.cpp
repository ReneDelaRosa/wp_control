//http://docs.opencv.org/3.1.0
/*
	28 Febrero 2018 22:23
	Se agregó Filtro Median Blur
*/
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;

class Edge_Detector
{
   	ros::NodeHandle nh1_;
  	image_transport::ImageTransport it1_;
  	image_transport::Subscriber image_sub_;
  	image_transport::Publisher image_pub_;
  	
	ros::Publisher mx_pub;
  	ros::Publisher my_pub;
  	ros::Publisher det_pub;

	int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;
	int cx, cy;
	int M_Blur;
	int B_MAX;

	int Thresh_Close;
	int Max_Thresh_C;			
	
	std::string windowName = "Original Image";
	std::string windowName1 = "HSV Image";

	std_msgs::Int32 msgx; //Momento en X
	std_msgs::Int32 msgy; //Momento en Y

	
public:
	//Constructor por defecto de la clase con lista de constructores
  	Edge_Detector() : it1_(nh1_){

    image_sub_ = it1_.subscribe("/bebop/image_raw", 1, &Edge_Detector::imageCb, this);
		//image_sub_ = it1_.subscribe("/usb_cam/image_raw", 1, &Edge_Detector::imageCb, this);

    mx_pub = nh1_.advertise<std_msgs::Int32>("/c_x",1);
    my_pub = nh1_.advertise<std_msgs::Int32>("/c_y",1);
	
   	H_MIN = 0;
	H_MAX = 255;
	S_MIN = 120;
	S_MAX = 255;
	V_MIN = 127;
	V_MAX = 255;
	cx=0; 
	cy=0; 
	M_Blur=4;
	B_MAX=20;
	Thresh_Close = 5;
	Max_Thresh_C = 20;  
  	}

	//Desctructor de la clase
  	~Edge_Detector(){
    		cv::destroyAllWindows();
	}

  	void imageCb(const sensor_msgs::ImageConstPtr& msg){

    		cv_bridge::CvImagePtr cv_ptr;
    		namespace enc = sensor_msgs::image_encodings;

   		 try{
      			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		}
    		catch (cv_bridge::Exception& e){
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}	

		detect_edges(cv_ptr->image);	
  	}

	void createTrackbars(){

		cv::namedWindow(windowName1, CV_WINDOW_AUTOSIZE);
		      
		cv::createTrackbar("H_MIN", windowName1, &H_MIN, H_MAX);
		cv::createTrackbar("H_MAX", windowName1, &H_MAX, H_MAX);
		cv::createTrackbar("S_MIN", windowName1, &S_MIN, S_MAX);
		cv::createTrackbar("S_MAX", windowName1, &S_MAX, S_MAX);
		cv::createTrackbar("V_MIN", windowName1, &V_MIN, V_MAX);
		cv::createTrackbar("V_MAX", windowName1, &V_MAX, V_MAX);
		cv::createTrackbar("MedianB", windowName1, &M_Blur,B_MAX);
		cv::createTrackbar("Thresh_Close", windowName1, &Thresh_Close, Max_Thresh_C);

	
	}

  	void detect_edges(cv::Mat img){
	
		cv::Mat src,HSV;
		cv::Mat threshold;
		cv::Mat threshold1;
		cv::Mat combine_image; 
		cv::Mat image_exit;


		vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
		vector<Vec4i> lines;
		
		int d = 0,r = 0,cy_min=0,cy_max=0;
		//double area = 0, area_max = 0;
		
		char str[200];
	
		img.copyTo(src);
		cv::Mat roi_src(src, Rect(200,0,240,368));//Rect(200,100,240,268))
		createTrackbars();

		cv::cvtColor(roi_src, HSV, CV_RGB2HSV_FULL);
		cv::inRange(HSV, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);
		cv::medianBlur(threshold,threshold,2*M_Blur + 1);
				
		cv::Mat Element = getStructuringElement(cv::MORPH_ELLIPSE,  Size( 2*Thresh_Close + 1, 2*Thresh_Close+1 ), Point( Thresh_Close, Thresh_Close ));
		cv::morphologyEx(threshold, combine_image, cv::MORPH_CLOSE, Element);
		
		findContours( combine_image, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

		if (contours.size() == 0){
			
			sprintf(str, "No detectado");
			putText(roi_src, str, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,0,255), 2);
			cv::imshow(windowName1, combine_image);
			cv::imshow(windowName, roi_src);
			cx = -1;	
			cy = -1;
			
			msgx.data = cx;
			msgy.data = cy;
			
			mx_pub.publish(msgx);
			my_pub.publish(msgy);
			cout<<"Vision "<<"x: "<<cx<<" y: "<<cy<<endl;
			cv::waitKey(3);
			
		}
		else{
			// Momentos
			vector<Moments> mu(contours.size() );
			// Centros de Masa
	  		vector<Point2f> mc( contours.size() );
	  		// Vector de centroides
	  		vector<int> v_cx(contours.size());
	  		vector<int> v_cy(contours.size());

	  		if(contours.size()>1){
		  		//Calcular los momentos y centroides
		  		for (r=0;r<contours.size();r++){
			  		mu[r] = moments( contours[r], false ); 
					mc[r] = Point2f( mu[r].m10/mu[r].m00 , mu[r].m01/mu[r].m00 );
					v_cx[r] = mu[r].m10/mu[r].m00;	
					v_cy[r] = mu[r].m01/mu[r].m00;
				}
			}	
			else{
				r=0;
				mu[r] = moments( contours[r], false ); 
				mc[r] = Point2f( mu[r].m10/mu[r].m00 , mu[r].m01/mu[r].m00 );
				v_cx[r] = mu[r].m10/mu[r].m00;	
				v_cy[r] = mu[r].m01/mu[r].m00;
			}

			//Obtener el indice de la cordenada cy mayor
			if(contours.size()>1){
				for(r=0;r<v_cy.size(); r++ ){
					cy_min = v_cy[r];
					if (cy_min > cy_max){
						cy_max = cy_min;
						d = r;
					}
				}
			}
			else{
				d=0;
			}

			cx=v_cx[d];
			cy=v_cy[d];

			cout<<"Vision "<<"x: "<<cx<<" y: "<<cy<<endl;

			//Dibuja contornos y centros de Masa				
			for(r=0;r<contours.size();r++){
				drawContours( roi_src, contours, r, cv::Scalar(0, 0, 255), 1, 8, hierarchy, 0, Point() );
				circle( roi_src, mc[r], 2, cv::Scalar(0, 255, 0),-1);
			}

			sprintf(str,"x = %d y = %d",cx,cy);
			putText(roi_src, str, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(20,255,0), 2);
			
			cv::imshow(windowName1, combine_image);
			cv::imshow(windowName, roi_src);
			
			msgx.data = cx;
			msgy.data = cy;
		
			mx_pub.publish(msgx);
			my_pub.publish(msgy);
			cv::waitKey(3);						
		}			
  	}	 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Edge_Detector");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
