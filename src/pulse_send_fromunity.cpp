#include <stdio.h>
#include <stdlib.h>

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

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
#include <unistd.h>	 /* UNIX standard function definitions */
#include <fcntl.h>	 /* File control definitions */
#include <errno.h>	 /* Error number definitions */
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
#define ACT_ID_RM 2 // 1
#define N_IMU_SH 2
#define SOGLIA_PARAM 0.9 // Define for vibrotactile algorithm
#define SOGLIA 0.04		 // Define for vibrotactile algorithm
#define SHIFT 5
#define SCALA 45
#define PWM_MAX_VALUE_DC 100
#define DEFAULT_FREQUENCY 1100
#define NUM_SECONDS_TOTAL 3
#define NUM_ACT 3

bool active = 1;
float newTime = 0;
auto start = high_resolution_clock::now();
auto stop = high_resolution_clock::now();

double time_to_save[DEFAULT_FREQUENCY * NUM_SECONDS_TOTAL];
// comm_settings comm_settings_imu;
// comm_settings comm_settings_act;
int n_imu = 2;
float imu_values[3 * N_IMU_SH];
float accelerometer_matrix[N_IMU_SH][3];
float matrice_reco_mod[N_IMU_SH * 3];
float acc_values_mod[N_IMU_SH][3];
float matrice_completa[3 * N_IMU_SH];

float vect_acc_filt_sum[N_IMU_SH];
short int input_act_sent[N_IMU_SH] = {0};
short int act_sent[NUM_ACT] = {0};

float matrice_reco[3 * N_IMU_SH];
float contatore[3 * N_IMU_SH];
int conta;
int input_act_old[N_IMU_SH] = {0};
// int trovato[N_IMU_SH] = {0,0};
// int massimo[N_IMU_SH] = {0,0};
int flag_imu = 0;
float conta_1 = 0;

// bandpass_filter_param

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
float A1 = (0.19227 / A0);
float A2 = (1.09197 / A0);
float A3 = (0.15811 / A0);
float A4 = (0.72636 / A0);
float B_0 = (0.06423 / A0);
float B1 = (0.0 / A0);
float B2 = (-0.12845 / A0);
float B3 = (0.0 / A0);
float B4 = (0.06423 / A0);

int SoftHand_id = 1;
int Pulse_id = 2;
short int inputs[2];
short int inputsPULSE[3];
bool initcall = false;

/* read accelerations */
using namespace Eigen;

double accmatrix;
double forcematrix;
short int currents[2];
short int pressure;

void readCallback(const std_msgs::Float64::ConstPtr &acc)
{

	initcall = true;
	// cout << "velocity:" << endl;
	// cout << accmatrix << endl;

	accmatrix = acc->data;

	// accmatrix.row(0) << acc->data[0], acc->data[1], acc->data[2], acc->data[3], acc->data[4], acc->data[5];
}

void readCallback2(const std_msgs::Float64::ConstPtr &force)
{
	initcall = true;
	forcematrix = force->data;
	// cout << "force:" << endl;

	// cout << forcematrix << endl;
}

void signal_callback_handler(int signum)
{ // Codice che viene eseguito quando fai CTRL+C
	initcall = false;
	inputsPULSE[0] = 0; // Reset inputs
	inputsPULSE[1] = 0;
	inputsPULSE[2] = 0;
	commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send reset to device
	commActivate(&comm_settings_t, Pulse_id, 0);
	std::cout << "\n- valve open - silicone chambers deflating-\n"
			  << std::endl;
	usleep(1000000);

	closeRS485(&comm_settings_t);
	std::cout << "\n - Execution stopped -  \n"
			  << std::endl;
	// Terminate program
	exit(signum);
}

int main(int argc, char **argv)
{

	// signal(SIGINT, signal_callback_handler);

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
	cout << "\n\t Pulse Device - For UNITY\n";
	cout << "\n\n------------------------------------------------------------------------\n";
	// Opening the port the device is connected to
	cout << "\n\n Opening COM ports to communicate with the Actuators.";

	const char *port_act = (char *)"/dev/ttyUSB0";

	openRS485(&comm_settings_t, port_act);

	commActivate(&comm_settings_t, Pulse_id, 1);

	usleep(500000);
	//--------------------------Variables for IMU-D321 code with PWM generators---------------------
	float acc_values_mod[n_imu_][3];
	// float vect_filt[n_imu_][3];
	float matrice_completa[3 * n_imu_];
	float matrice_reco[3 * n_imu_];
	float matrice_reco_mod[3 * n_imu_];
	float vect_acc_filt_sum[n_imu_];
	float soglia = 0.04; // 0.01;
	int shift = 1200;	 // 50;
	float scala = 4.5;	 // 25;
	int conta = 0;
	int contatore[3 * n_imu_];

	cout << "\n\n------------------------------------------------------------------------\n";
	cout << "\n\n Pre-operation is completed, now to start the experiment press ENTER\n";
	cin.get();

	//---------------------------- manage time------------------------------------

	auto start = std::chrono::system_clock::now();
	auto end = start;
	std::chrono::duration<double> elapsed_mseconds_t = (end - start) * 1000;
	int i = 0;
	start = std::chrono::system_clock::now();
	end = start;
	elapsed_mseconds_t = (end - start) * 1000;

	//------------------------------PUMP and VT send signals from file--------------------
	double oldforce = 0;
	int oldi = 0;

	double friction;

	// open to deflate the silicone chambers
	commActivate(&comm_settings_t, Pulse_id, 0);
	std::cout << "\n- valve open - silicone chambers deflating-\n"
			  << std::endl;
	usleep(1000000);

	// CLOSE THE VALVE sending '1'
	commActivate(&comm_settings_t, Pulse_id, 1);
	usleep(1000);
	std::cout << "\n- closed -\n"
			  << std::endl;
	usleep(1000000);

	while (ros::ok())
	{

		if (initcall)
		{

			//------------controllo dati---
			/*	cout << "------pressure-----" << endl;
				cout << forcematrix  << endl;
				cout << "------friction-----" << endl;
				cout << accmatrix << endl;

	*/
			friction = abs(accmatrix);
			inputsPULSE[1] = friction * 90;

			// cout << accmatrix << endl;

			//-------------Compute INPUTS-------------old
			/*
						inputsPULSE[0] = forcematrix * 2; // forcematrix * 100;	 // PUMP input before *8 64
						inputsPULSE[1] = friction * 240;  // abs(friction * 20); // VT1 input
						inputsPULSE[2] = 0;				  // abs(friction * 20); // VT2 input

						// THRESHOLDS for the values coming from unity ------------------------------------------------------

						if (inputsPULSE[0] > 80)
						{
							inputsPULSE[0] = 80;
						}
						if (inputsPULSE[0] < 0)
						{
							inputsPULSE[0] = 0;
						}

						if (inputsPULSE[1] > 80)
						{
							inputsPULSE[1] = 80;
						}
						if (inputsPULSE[1] > 0)
						{
							inputsPULSE[1] = inputsPULSE[1] + 10;
						}

						if (inputsPULSE[1] < 0)
						{
							inputsPULSE[1] = 0;
						}

						if (inputsPULSE[2] > 40)
						{
							inputsPULSE[2] = 40;
						}
						if (inputsPULSE[2] < 0)
						{
							inputsPULSE[2] = 0;
						}
						*/
			//-------------------compute inputs new
			//----------force--------------
			if (forcematrix < 0.5)
				inputsPULSE[0] = 0;
			if (forcematrix > 0.5 && forcematrix < 10)
				inputsPULSE[0] = 20;
			if (forcematrix > 10)
				inputsPULSE[0] = 60;

			//------------velocity

			if (inputsPULSE[1] > 80)
			{
				inputsPULSE[1] = 80;
			}
			if (inputsPULSE[1] > 0)
			{
				inputsPULSE[1] = inputsPULSE[1] + 10;
			}

			if (inputsPULSE[1] < 0)
			{
				inputsPULSE[1] = 0;
			}

			inputsPULSE[2] = 0; // VT2

			//------------------silicone chambers: pump, valve and pressure sensor settings

			if (i == 0) // se primo giro azzera tutto
			{
				oldforce = 0;
				inputsPULSE[0] = 0; // Reset inputs
				inputsPULSE[1] = 0;
				inputsPULSE[2] = 0;
				commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send reset to device
				commActivate(&comm_settings_t, Pulse_id, 0);
				std::cout << "\n- valve open - silicone chambers deflating-\n"
						  << std::endl;
				usleep(1000000);
			}

			else

			{
				//--------------------------------------------
				if (inputsPULSE[0] == 0) // se zero sgonfia
				{

					// open to deflate the silicone chambers
					commActivate(&comm_settings_t, Pulse_id, 0);
					// std::cout << "\n- valve open - silicone chambers deflating-\n"<< std::endl;
					usleep(10000);
					oldforce = inputsPULSE[0];
					inputsPULSE[0] = 0;
					oldi = 0;

				}
				else if (inputsPULSE[0] > oldforce) // se valore maggiore di quello prima di X allora gonfia ancora e salva nuovo valore
				{
					// close to inflate the silicone chambers
					commActivate(&comm_settings_t, Pulse_id, 1);
					// std::cout << "\n- vale closed - silicone chambers inflating\n"<< std::endl;
					usleep(10000);
					oldforce = inputsPULSE[0];
					oldi = 0;

				}

				else if (inputsPULSE[0] = oldforce && inputsPULSE[0] != 0) // se valore uguale a prima + o - di X valvola chiusa e non gonfiare
				{	
					commActivate(&comm_settings_t, Pulse_id, 1);
					// std::cout << "\n- vale closed - silicone chamber not inflating nor deflating \n" << std::endl;
					usleep(10000);
					// oldforce = inputsPULSE[0];
					oldi = oldi +1; 

					if (oldi > 2)
					{
						// open to deflate the silicone chambers
						commActivate(&comm_settings_t, Pulse_id, 1);
						// std::cout << "\n- valve open - silicone chambers deflating-\n"<< std::endl;
						usleep(10000);
						oldforce = inputsPULSE[0];
						inputsPULSE[0] = 0;
					}

					
				}
				else if (inputsPULSE[0] < oldforce && inputsPULSE[0] != 0) // se valore minore di quello prima di X allora sgonfia ancora e salva nuovo valore
				{
					// close to inflate the silicone chambers
					commActivate(&comm_settings_t, Pulse_id, 0);
					// std::cout << "\n- vale closed - silicone chambers inflating\n"<< std::endl;

					usleep(10000);
					oldforce = inputsPULSE[0];
					inputsPULSE[0] = 0;
					oldi = 0;
				}

				//------------------------- old but works
				/*

												if (inputsPULSE[0] < oldforce && i > oldi + 1)
													{
														// open to deflate the silicone chambers
														commActivate(&comm_settings_t, Pulse_id, 0);
														// std::cout << "\n- valve open - silicone chambers deflating-\n"<< std::endl;
														usleep(10000);
														oldforce = inputsPULSE[0];
														inputsPULSE[0] = 0;
														oldi = i;
													}

													if (inputsPULSE[0] > oldforce)
													{
														// close to inflate the silicone chambers
														commActivate(&comm_settings_t, Pulse_id, 1);
														// std::cout << "\n- vale closed - silicone chambers inflating\n"<< std::endl;

														usleep(10000);
														oldforce = inputsPULSE[0];
													}

													if (inputsPULSE[0] == oldforce && i > oldi + 3)
													{
														commActivate(&comm_settings_t, Pulse_id, 1);
														// std::cout << "\n- vale closed - silicone chamber not inflating nor deflating \n" << std::endl;
														usleep(10000);
														oldforce = inputsPULSE[0];
														inputsPULSE[0] = 0;
														oldi = i;
													}
				*/

				//-----------------------------------
			}

			i = i + 1;
			// cout << i << endl;
			/*cout << "------INPUTs-----" << endl;

			cout << inputsPULSE[0] << endl;
			cout << inputsPULSE[1] << endl;
			cout << inputsPULSE[2] << endl;*/

			// cout << inputsPULSE[0] << endl;
			// cout << inputsPULSE[1] << endl;
			// cout << inputsPULSE[2] << endl;

			// Settings ALL inputs N.B LA FUNZIONE È commSetInputsPULSE, diversa da commSetInputs che invece serve a controllare il motore della mano
			commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send inputs to device
			usleep(10000);

			// THRESHOLDS for the pressure value from the silicone chambers------------------------------------------------------

			commGetCurrents(&comm_settings_t, Pulse_id, currents);
			pressure = currents[0]; // il valore in uscità è in KPa*100 --> 2500 è 25KPa
			// printf("Pressure value: %hd\t\n ", pressure);
			fflush(stdout);
			usleep(10000);

			if (pressure > 650) // 950 old
			{
				cout << "pressure limit---STOP NOW" << endl;
				/*commActivate(&comm_settings_t, Pulse_id, 0);
				std::cout << "\n- valve open - silicone chambers deflating-\n"
						  << std::endl;
				usleep(1000000);*/
				commActivate(&comm_settings_t, Pulse_id, 1);
				usleep(1000);
				std::cout << "\n- vale closed - silicone chambers nor deflating nor inflating\n"
						  << std::endl;
				inputsPULSE[0] = 0;
				commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send inputs to device



				usleep(1000000);
			}

			if (pressure > 1100) // 950 old
			{
				cout << "pressure limit---deflating NOW" << endl;
				commActivate(&comm_settings_t, Pulse_id, 0);
				std::cout << "\n- valve open - silicone chambers deflating-\n"
						  << std::endl;
				usleep(1000000);
				commActivate(&comm_settings_t, Pulse_id, 1);
				usleep(1000);
				std::cout << "\n- vale closed - silicone chambers nor deflating nor inflating\n"
						  << std::endl;
				inputsPULSE[0] = 0;
				commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send inputs to device



				usleep(1000000);
			}

			usleep(1000);

			/*while (elapsed_mseconds_t.count() < 5000)
			{
				// imu1[i]= abs(accmatrix(0) + accmatrix(1) + accmatrix(2));

				// imu2[i]= abs(accmatrix(3) + accmatrix(4) + accmatrix(5));
				// cout << forcematrix.size() << endl;

				cout << accmatrix[1] << endl;
				//

				// N.B LA FUNZIONE È commSetInputsPULSE, diversa da commSetInputs che invece serve a controllare il motore della mano
				// commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send inputs to device

				end = std::chrono::system_clock::now();
				elapsed_mseconds_t = (end - start) * 1000;
				i = i + 1;
				 // delay between samples
			}*/
		}
		//------------------------------ set all to 0 berfore closing

		inputsPULSE[0] = 0; // Reset inputs
		inputsPULSE[1] = 0;
		inputsPULSE[2] = 0;
		commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send reset to device
																	 // open to deflate the silicone chambers

		ros::spinOnce();   // Need to call this function often to allow ROS to process incoming messages
		loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
	}

	inputsPULSE[0] = 0; // Reset inputs
	inputsPULSE[1] = 0;
	inputsPULSE[2] = 0;
	commSetInputsPULSE(&comm_settings_t, Pulse_id, inputsPULSE); // Send reset to device
	commActivate(&comm_settings_t, Pulse_id, 0);
	std::cout << "\n- valve open - silicone chambers deflating-\n"
			  << std::endl;
	usleep(1000000);

	closeRS485(&comm_settings_t);
	std::cout << "\n- End of Code -\n"
			  << std::endl;

	return 0;
}
