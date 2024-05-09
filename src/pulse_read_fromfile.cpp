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



int main(int argc, char **argv)
{
	//signal(SIGINT, int_handler); 

	int num_measure = 1956;

	uint8_t aux_string[2000];
	uint8_t PARAM_SLOT_BYTES    = 50;
	uint8_t num_imus_id_params  = 6; 
	uint8_t num_mag_cal_params  = 0;
	uint8_t first_imu_parameter = 2;
	int     v                   = 0;
	float*  imu_read;
    int      n_imu_;
    uint8_t* imu_table_;
    uint8_t* mag_cal_;

    ros::init(argc,argv,"acc_read");  // Initiate new ROS node named "acc_read"

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle node;  

	ros::Publisher acc_pub = node.advertise<std_msgs::Float64MultiArray>("acc", 1); //nome topic 
	//ros::Publisher conta_loop_pub = node.advertise<std_msgs::Int16>("conta_loop",1000);


    std_msgs::Float64MultiArray acc; // modificare con il valore che dovrò inviare all'act_send
	//std_msgs::Int16 conta_loop;

	acc.data.resize(5,3); 

	ros::Publisher force_pub = node.advertise<std_msgs::Float64MultiArray>("force", 1); //nome topic 

    std_msgs::Float64MultiArray force; // modificare con il valore che dovrò inviare all'act_send
	//std_msgs::Int16 conta_loop;

	force.data.resize(1,1); 

    n_imu_=1;

	/*
	ros::Publisher accel_pub;
	accel_pub = node.advertise<acc_read_ss::Messaggio>("accel", 10);
	acc_read_ss::Messaggio acceleration;

	acceleration.signal.resize(5);
	*/

	ros::Rate loop_rate(1300);  // Desired rate to turn in Hz	

	//ros::Rate loop_rate(1/0.005);

    //----------- file to save

     // -----------Create a vector of <string> to store the result
    vector<string> result;

    float soglia_param ;
    int shift;//940 420     750 300 0.6
    float scala;//70 550 0.5}
   
    //----------------variables 
    float soglia = 0.04; // soglia outliers

    soglia_param = 0.9; // soglia dopo fabs
    shift = 1200;//940;
    scala =5.49;//scala per attuatore 4.6 senza soglia param e senza filtro;5.49 con soglia param e senza filtro bpf
   

    vector < double > result_d;
    vector < double > acc_xyz;
    float vect_filt[n_imu_][3];
    float vect_acc_filt_sum[n_imu_];
    
    int32_t input_act[1] = {0};
    int lin=0;

    int conta_finestra_para = 5;

    int32_t input_act_sent[n_imu_] = {0};
    int32_t input_act_old[n_imu_]  = {0};

    bool trovato[n_imu_] = {false};
    int massimo[n_imu_] = {0};
    int conta_finestra[n_imu_] = {conta_finestra_para};
    int contatore[3*n_imu_];
    float acc_values_mod[n_imu_][3];
    float matrice_reco[3*n_imu_];
    float matrice_reco_mod[3*n_imu_]; 
    float matrice_completa[3*n_imu_];     
    int i = 0;

    string filename = "/home/aivani/At-JND_HAPT/AT&JND/JND/stimuli/Quelliveri/pronti/p1000_1.csv";
    string filename2 = "/home/aivani/catkin_ws/src/pulse/src/force.csv";

    //--------------Create an input filestream
    ifstream myFileacc(filename);



    // Make sure the file is open
    if(!myFileacc.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    string line;
    string value;
    lin=0;
    vect_acc_filt_sum[0]=0;


    i=0;


    ifstream myFileforce(filename2);



    // Make sure the file is open
    if(!myFileforce.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    string line2;
    string value2;
    


    // Read data, line by line
    if(myFileacc.good() && myFileforce.good())
    {
        // Extract the first line in the file


        while((std::getline(myFileacc, line))&& (std::getline(myFileforce, line2)))
        {

            // Create a stringstream from line
            stringstream ss(line);
            stringstream ss2(line2);
           // column index---------from cv file of NI---acc values in 81, 82 83 
            int index = 0;
            int j=0; 
            
            // Extract each value from the line 
            while ((std::getline(ss, value, ';')) || (std::getline(ss2, value2, ';')))
                {

            //cout << ss2 << endl;
            // Save only accelerations values
                if ((index == 0) || (index== 1) || (index ==2) )
                {
                        //cout << value2 << endl;            

                        double value_d = std::stod(value);
                       
                        //cout <<"acc_"<<index<<": "<< value_d << endl;
                        acc.data[i,index] = value_d;
                }
               else if ((index == 3) || (index== 4) || (index ==5) && n_imu_==2)

                {
                        i=1;
                        double value_d = std::stod(value);
                        cout << "HERE" << endl; 

                        cout <<"acc_"<<index<<": "<< value_d << endl;
                        acc.data[i,index] = value_d;
                        i=0;

                }


                if ((j == 0))
                {
                        
                       
                         cout << "HERE" << endl; 

                        cout << value2 << endl;
                        double value_d2 = std::stod(value2);

                        //out <<"acc_"<<index<<": "<< value_d << endl;
                        force.data[j,j] = value_d2;
                        

                }



                //cout << "----------" << endl;
                    
                acc_pub.publish(acc);
                force_pub.publish(force);


                ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
		        loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate
                j++;
                index++;
                }
				
			i ++; 	
		}

    }

}

