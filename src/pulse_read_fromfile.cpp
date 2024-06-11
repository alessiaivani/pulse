#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <eigen3/Eigen/Dense>

#include <acc_read/Messaggio.h>

#include <sstream>

#include "Force_feedback.h"

#include <chrono>
#include <thread>

#include "iostream"
#include <math.h>
#include <fstream>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <ctime>


using namespace std::chrono;
using namespace Eigen;

int main(int argc, char **argv)
{
	//signal(SIGINT, int_handler); 

       ros::init(argc,argv,"acc_read");  // Initiate new ROS node named "acc_read"

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle node;  

	ros::Publisher acc_pub = node.advertise<std_msgs::Float64MultiArray>("acc", 1); //nome topic 
	//ros::Publisher conta_loop_pub = node.advertise<std_msgs::Int16>("conta_loop",1000);


    std_msgs::Float64MultiArray acc; // modificare con il valore che dovrò inviare all'act_send
	//std_msgs::Int16 conta_loop;

	acc.data.resize(6); 

	ros::Publisher force_pub = node.advertise<std_msgs::Float64MultiArray>("force", 1); //nome topic 

    std_msgs::Float64MultiArray force; // modificare con il valore che dovrò inviare all'act_send
	//std_msgs::Int16 conta_loop;


	force.data.resize(1); 

	ros::Rate loop_rate(1000);  // Desired rate to turn in Hz	


    int i,j = 0;
    //-----------------load file------------------
    string filename = "/home/aivani/catkin_ws/src/pulse/src/imu_file.csv";
    string filename2 = "/home/aivani/catkin_ws/src/pulse/src/sine.csv";

    //--------------Create an input filestream
    
    ifstream myFileforce(filename2);
    // Make sure the file is open
    if(!myFileforce.is_open()) throw std::runtime_error("Could not open file");
    // Helper vars
    string line2;
    string value2;
    std::vector<double> sine(10000);
    vector<vector<string>> content2;
    vector<string> row2;


    
    if(myFileforce.is_open())
    {
        while(getline(myFileforce, line2))
        {
            row2.clear();
            stringstream str(line2);
            while(getline(str, value2, ','))

                //cout << value2 << endl; 
                row2.push_back(value2);
                content2.push_back(row2);

                
        }
         cout<<"\n Data imported succesfully!\n\n";
    }
    else{
        cout<<"Could not open the file\n";
        }
 
    for(int i = 0; i < content2.size(); i++)
    {
        sine[i] = std::stod(content2[i][0]);
     //  cout << sine[i] << endl; 
    }


    ifstream myFileacc(filename);
    // Make sure the file is open
    if(!myFileacc.is_open()) throw std::runtime_error("Could not open file");
    // Helper vars
    string line;
    string value;
    vector<vector<string>> content;
    vector<string> row;
    vector<vector<string>> content3;
    vector<string> row3;


   double vibroT1 [10000][3];
   double vibroT2 [10000][3];



    vector<string> result;
    vector < double > result_d;

    vector<string> result2;
    vector < double > result_d2;

    //--------------Create an input filestream

    // Make sure the file is open
    if(!myFileacc.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    int lin=0;
    // Read data, line by line
    if(myFileacc.good())
    {
        // Extract the first line in the file
        while(std::getline(myFileacc, line))
        {

            // Create a stringstream from line
            stringstream ss(line);

            //cout << ss.str() << endl;

            // column index---------from cv file of NI---acc values in 81, 82 83 
            int index = 0;
            
            // Extract each value from the line 
            while (std::getline(ss, value, ',')){

                //cout << index << endl;
                if (lin!=0){
                    
                    // Save only accelerations values
                    if ((index == 3) || (index== 4) || (index == 5)){
                        //cout << value << endl;
                        result.push_back(value);
                        double value_d = std::stod(value);
                        //cout << value_d << endl;
                        result_d.push_back(value_d);

                    }

                    if ((index ==6) || (index== 7) || (index == 8)){
                        //cout << value << endl;
                        result2.push_back(value);
                        double value_d2 = std::stod(value);
                        //cout << value_d << endl;
                        result_d2.push_back(value_d2);

                    }
                }
                
                index++;
             
            
                
            }
            lin++;



        }   
    // Close file
    myFileacc.close();

        
        cout<<"\n Data imported succesfully!\n\n";
    }
    else{
        cout<<"Could not open the file\n";
        }
       

    
    cout << result_d2.size() << endl; 
 
    for(int i = 0; i < result_d.size()/3; i++){
        vibroT1[i][0] = (result_d[0+i*3]);
        vibroT1[i][1]  = (result_d[1+i*3]);
        vibroT1[i][2]  = (result_d[2+i*3]);
        vibroT2[i][0]  = (result_d2[0+i*3]);
        vibroT2[i][1]  = (result_d2[1+i*3]);
        vibroT2[i][2]  = (result_d2[2+i*3]);

      //cout << vibroT1[i][0] << endl; 

               
	//	vibroT2[i] = std::stod(content[i][1]);
    }


    
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	high_resolution_clock::time_point t2 = high_resolution_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2-t1);

	auto start = std::chrono::system_clock::now();
	auto end = start;
	std::chrono::duration<double> elapsed_mseconds_t = (end - start) * 1000;


    start = std::chrono::system_clock::now();
	end = start;
    elapsed_mseconds_t = (end - start) * 1000;


//    cout << "-------------------------------------------" << endl;
    
	for (i=0; elapsed_mseconds_t.count() < 4000; i++) {


        acc.data[0] = vibroT1[i][0];
        acc.data[1] = vibroT1[i][1];
        acc.data[2] = vibroT1[i][2];

        acc.data[3] = vibroT2[i][0];
        acc.data[4] = vibroT2[i][1];
        acc.data[5] = vibroT2[i][2];

       //cout <<  acc.data[3]<< endl; 
        //cout <<  i<< endl; 

        //cout << vibroT1[i][0] << endl; 


        acc_pub.publish(acc);
        // Need to call this function often to allow ROS to process incoming messages 

        
        force.data[0] = sine[i];
        force_pub.publish(force);


        end = std::chrono::system_clock::now();
        elapsed_mseconds_t = (end - start) * 1000;
        i = i + 1;
        usleep(100); //delay between sam

    ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
	loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate

      
	}

                    
          

cout << "end" << endl;
		

    


     }
