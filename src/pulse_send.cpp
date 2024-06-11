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


#include <chrono>
#include <thread>

#include <stdint.h>
#include <ctype.h>
#include <time.h>
#include <thread>
#include <fstream>
#include <string>
#include <vector>
#if (defined(_WIN32) || defined(_WIN64))
    #include <windows.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64))
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <sys/ioctl.h>
    #include <dirent.h>
    #include <sys/time.h>
    #include <stdlib.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__))
    #include <linux/serial.h>
#endif
#include "qbmove_communications.h" 
#include "cp_communications.h"
#include "Force_feedback.h"

#include <math.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <chrono> 
using namespace std::chrono;
using namespace std;

#define SENS_ID 1
#define ACT_ID_RM 2//1
#define N_IMU_SH 2
#define SOGLIA_PARAM            0.9     // Define for vibrotactile algorithm
#define SOGLIA                  0.04    // Define for vibrotactile algorithm
#define SHIFT                   5
#define SCALA                   45
#define PWM_MAX_VALUE_DC        100
#define DEFAULT_FREQUENCY 1100
#define NUM_SECONDS_TOTAL 3
#define NUM_ACT 3

bool active = 1;
float newTime = 0;
auto start = high_resolution_clock::now();
auto stop = high_resolution_clock::now(); 
		 

double time_to_save[DEFAULT_FREQUENCY * NUM_SECONDS_TOTAL];
//comm_settings comm_settings_imu;
//comm_settings comm_settings_act;
int n_imu=2;
float imu_values[3*N_IMU_SH];
float accelerometer_matrix[N_IMU_SH][3];
float matrice_reco_mod[N_IMU_SH*3];	
float acc_values_mod[N_IMU_SH][3];
float matrice_completa[3*N_IMU_SH];	

float vect_acc_filt_sum[N_IMU_SH];
short int input_act_sent[N_IMU_SH] = {0};
short int act_sent[NUM_ACT] = {0};

float matrice_reco[3*N_IMU_SH];
float contatore[3*N_IMU_SH];
int conta;
int input_act_old[N_IMU_SH] = {0};
//int trovato[N_IMU_SH] = {0,0};
//int massimo[N_IMU_SH] = {0,0};
int flag_imu=0;
float conta_1=0;



//bandpass_filter_param

float s1, s2, s3, s4, s5, s6, s7, s8, s9, s10;
/*
float vect_acc[N_IMU_SH][3];            // Last Sample (Filter input)
float vect_acc_old[N_IMU_SH][3];         // Sample one step back 
float vect_acc_old2[N_IMU_SH][3];        // Sample two steps back
float vect_acc_old3[N_IMU_SH][3];        // Sample three steps back
float vect_acc_old4[N_IMU_SH][3];
    
float vect_acc_filt[N_IMU_SH][3];        // Filtered last sample (Filter Output)
float vect_acc_old_filt[N_IMU_SH][3];    // Filter output one step back
float vect_acc_old2_filt[N_IMU_SH][3];   // Filter output two steps back
float vect_acc_old3_filt[N_IMU_SH][3];   // Filter output three steps back
float vect_acc_old4_filt[N_IMU_SH][3];*/
float vect_filt[N_IMU_SH][3];

 #define A0 1.0
 float A1   = (0.19227    / A0);
 float A2   = (1.09197    / A0);
 float A3   = (0.15811    / A0);
 float A4   = (0.72636    / A0);
 float B_0  = (0.06423    / A0);
 float B1   = (0.0        / A0);
 float B2   = (-0.12845   / A0);  
 float B3   = (0.0        / A0);
 float B4   = (0.06423    / A0);


int SoftHand_id = 1;
int Pulse_id = 2;
short int inputs[2];
short int inputsPULSE[3];


/* read accelerations */
using namespace Eigen;

typedef Matrix<float, 1,6> Matrix53f;

Matrix53f accmatrix;

typedef Matrix<float,1,1> Matrix1f;

Matrix1f forcematrix;

void readCallback(const std_msgs::Float64MultiArray::ConstPtr &acc)
{

	accmatrix.row(1) << acc->data[0], acc->data[1], acc->data[2], acc->data[3], acc->data[4], acc->data[5];

}


void readCallback2(const std_msgs::Float64MultiArray::ConstPtr &force)
{

		forcematrix.row(1)  << force->data[0];
	
}


int main(int argc, char **argv)
{

	//signal(SIGINT, signal_callback_handler);

	const int n_imu_ = 2;

	int pump; 

   //---------------------------------------------NODEs SUB--------------------------------------
	ros::init(argc, argv, "pulse_send"); // Initiate new ROS node named "act_send_ss"

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle node;

	ros::Subscriber acc_sub = node.subscribe("acc", 1, readCallback);
	ros::Subscriber force_sub = node.subscribe("force", 1, readCallback2);

	ros::Publisher act_pub = node.advertise<std_msgs::Int16>("act", 1, true);

	std_msgs::Int16 act;

	ros::Rate loop_rate(1000); // Desired rate to turn in Hz

	//--------------------------------------------Device preparation----------------------------
	cout << "\n\t Pulse Device\n";
	cout << "\n\n------------------------------------------------------------------------\n";
	// Opening the port the device is connected to
	cout << "\n\n Opening COM ports to communicate with the Actuators.";

	const char *port_act = (char *)"/dev/ttyUSB0";

	openRS485(&comm_settings_act, port_act);

	commActivate(&comm_settings_act, ACT_ID_RM, 1);

	usleep(500000);
 //--------------------------Variables for IMU-D321 code with PWM generators---------------------
	float acc_values_mod[n_imu_][3];
	//float vect_filt[n_imu_][3];
	float matrice_completa[3 * n_imu_];
	float matrice_reco[3 * n_imu_];
	float matrice_reco_mod[3 * n_imu_];
	float vect_acc_filt_sum[n_imu_];
	float soglia = 0.04; // 0.01;
	int shift = 1200;	  // 50;
	float scala = 4.5; // 25;
	int conta = 0;
	int contatore[3 * n_imu_];

	cout << "\n\n------------------------------------------------------------------------\n";
	cout << "\n\n Pre-operation is completed, now to start the experiment press ENTER\n";
	cin.get();


	//---------------------------- manage time------------------------------------
				
	auto start = std::chrono::system_clock::now();
	auto end = start;
	std::chrono::duration<double> elapsed_mseconds_t = (end - start) * 1000;
	int i=0; 
	start = std::chrono::system_clock::now();
	end = start;
    elapsed_mseconds_t = (end - start) * 1000;

	//------------------------------PUMP and VT send signals from file--------------------

	std::vector<double> imu1(10000);
	std::vector<double> imu2(10000);



	while(elapsed_mseconds_t.count() < 5000) {

		imu1[i]= abs(accmatrix(0) + accmatrix(1) + accmatrix(2));

		imu2[i]= abs(accmatrix(3) + accmatrix(4) + accmatrix(5));


        inputsPULSE[0] = forcematrix[i] * 100; // PUMP input
		inputsPULSE[1] = imu1[i] * 10; // VT1 input
        inputsPULSE[2] = imu2[i] * 10; // VT2 input 

        //N.B LA FUNZIONE È commSetInputsPULSE, diversa da commSetInputs che invece serve a controllare il motore della mano
        commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send inputs to device

        end = std::chrono::system_clock::now();
        elapsed_mseconds_t = (end - start) * 1000;
        i = i + 1;
        usleep(1000); //delay between samples
	}

	inputsPULSE[0] = 0; // Reset inputs
    inputsPULSE[1] = 0;
	inputsPULSE[2] = 0;

    commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send reset to device

  	closeRS485( &comm_settings_t );
   	std::cout << "\n- End of Code -\n"  << std::endl;
		
	/*	--------------------------- PUMP and VT send signals with old code-----------------------
	while(active){
	for (int i = 0; i <3; i++) {
			
			//cout << i << endl;
		
				accelerometer_matrix[i][0] = accmatrix(i, 0);
				accelerometer_matrix[i][1] = accmatrix(i, 1);
				accelerometer_matrix[i][2] = accmatrix(i, 2);
				//filtro <<accelerometer_matrix[i][0] << "," <<accelerometer_matrix[i][1] << "," <<accelerometer_matrix[i][2] <<endl;
				//cout <<accelerometer_matrix[1][0] << "," <<accelerometer_matrix[1][1] << "," <<accelerometer_matrix[1][2] <<endl;

			OutliersDeletion(i);
       		SignalFiltering(i);
			//filtro << matrice_completa[3*i+0] << matrice_completa[3*i+1] <<matrice_completa[3*i+2] <<endl;
       		SingleAxisMapping(i); 
			//if (i==0)
			//map << vect_acc_filt_sum[0] << endl;
       		ActuatorsInputComputation(i);																	
			conta_1++;

		//	air_chambers_control(i);

			

			//cout << "input 0: " << input_act_sent[0] << endl;// Further signal manipulation 
			//cout << "input 1: " << input_act_sent[1] << endl;
			
			act_sent[0]=pump; // camerine

			act_sent[1]=input_act_sent[0];// imu 0
			act_sent[2]=input_act_sent[1];// imu 1


			//commSetInputs(&comm_settings_act, ACT_ID_RM, act_sent);		

			ros::spinOnce();     // Need to call this function often to allow ROS to process incoming messages 
		    loop_rate.sleep();   // Sleep for the rest of the cycle, to enforce the loop rate

		}



		usleep(1000);

	}

	*/
	
	return 0;
}

