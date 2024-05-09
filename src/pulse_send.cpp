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

//#include "qbmove_communications.cpp" DA AGGIUNGERE DOPO
//#include "cp_communications.cpp"
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

/* read accelerations */
using namespace Eigen;

typedef Matrix<float, 5, 3> Matrix53f;

Matrix53f accmatrix;

typedef Matrix<float,1,1> Matrix1f;

Matrix1f focematrix;

void readCallback(const std_msgs::Float64MultiArray::ConstPtr &acc)
{
	for (int k = 0; k < 5; k++)
	{
		accmatrix.row(k) << acc->data[k,0], acc->data[k,1], acc->data[k,2];
	}
}


void readCallback2(const std_msgs::Float64MultiArray::ConstPtr &force)
{
	for (int p = 0; p < 5; p++)
	{

		focematrix.row(p) << force->data[0];
	}
}
/*
void int_handler(int sig){

	short int inputs[2] = {0,0};
	commSetInputs(&comm_settings_act, ACT_ID_RM, inputs);
	closeRS485(&comm_settings_act);

	//cout << "here" <<  endl;
	active = 0;
}
*/
void OutliersDeletion(uint8_t i){
   
    if (conta_1 < 2){	
		acc_values_mod[i][0] = accelerometer_matrix[i][0];
		acc_values_mod[i][1] = accelerometer_matrix[i][1];
		acc_values_mod[i][2] = accelerometer_matrix[i][2];

	}

	else {
        
		if (contatore[3*i+0] > 2)
		{
			acc_values_mod[i][0] = accelerometer_matrix[i][0];
			contatore[3*i+0] = 0;
		}
		else
		{
              
            
			if ((fabs(accelerometer_matrix[i][0] - matrice_reco[3*i+0]) > SOGLIA )||( fabs(accelerometer_matrix[i][0] - matrice_reco[3*i+0]) < 0.000001))
			{
				acc_values_mod[i][0] = matrice_reco_mod[3*i+0];
				contatore[3*i+0] =contatore[3*i+0] + 1;
			}
			else if (fabs(accelerometer_matrix[i][0] - matrice_reco[3*i+0]) <= 0.04)
			{
				acc_values_mod[i][0] = accelerometer_matrix[i][0];
				contatore[3*i+0] = 0;
			}
		}


		if (contatore[3*i+1] > 2)
		{
			acc_values_mod[i][1] = accelerometer_matrix[i][1];
			contatore[3*i+1] = 0;
		}
		else
		{
			if (fabs(accelerometer_matrix[i][1] - matrice_reco[3 * i + 1]) > SOGLIA || fabs(accelerometer_matrix[i][1] - matrice_reco[3 * i + 1]) < 0.000001)
			{
				acc_values_mod[i][1] = matrice_reco_mod[3 * i + 1];
				contatore[3*i+1] += 1;
			}
			else if (fabs(accelerometer_matrix[i][1] - matrice_reco[3 * i + 1]) <= SOGLIA)
			{
				acc_values_mod[i][1] = accelerometer_matrix[i][1];
				contatore[3*i+1] = 0;
			}
		}

		if (contatore[3*i+2] > 2)
		{
			acc_values_mod[i][2] = accelerometer_matrix[i][2];
                

			contatore[3*i+2] = 0;
            
		}
		else
		{
			if (fabs(accelerometer_matrix[i][2] - matrice_reco[3 * i + 2]) > SOGLIA || fabs(accelerometer_matrix[i][2] - matrice_reco[3 * i + 2]) < 0.000001)
			{
				acc_values_mod[i][2] = matrice_reco_mod[3 * i + 2];
                    

				contatore[3*i+2] += 1;
			}
			else if (fabs(accelerometer_matrix[i][2] - matrice_reco[3 * i + 2]) <= SOGLIA)
			{
				acc_values_mod[i][2] = accelerometer_matrix[i][2];
                    

				contatore[3*i+2] = 0;
			}
		}
			}

			matrice_reco[3*i+0] = accelerometer_matrix[i][0];	
			matrice_reco[3*i+1] = accelerometer_matrix[i][1];
			matrice_reco[3*i+2] = accelerometer_matrix[i][2];

			matrice_reco_mod[3*i+0] = acc_values_mod[i][0];
			matrice_reco_mod[3*i+1] = acc_values_mod[i][1];
			matrice_reco_mod[3*i+2] = acc_values_mod[i][2];
            



}

/*******************************************************************************
* Function Name: BandPassFilter
*********************************************************************************/
float BandPassFilter(float new_value, int idx, int dir){

    vect_acc_old4[idx][dir] = vect_acc_old3[idx][dir];
    vect_acc_old3[idx][dir] = vect_acc_old2[idx][dir];
    vect_acc_old2[idx][dir] = vect_acc_old[idx][dir];
    vect_acc_old[idx][dir] = vect_acc[idx][dir];
    
    vect_acc_old4_filt[idx][dir] = vect_acc_old3_filt[idx][dir];
    vect_acc_old3_filt[idx][dir] = vect_acc_old2_filt[idx][dir];
    vect_acc_old2_filt[idx][dir] = vect_acc_old_filt[idx][dir];
    vect_acc_old_filt[idx][dir] = vect_acc_filt[idx][dir];
  
    vect_acc[idx][dir] = new_value;

    s1 = ( B_0 ) * ( vect_acc[idx][dir] );
    s2 = ( B1 ) * ( vect_acc_old[idx][dir] );
    s3 = ( B2 ) * ( vect_acc_old2[idx][dir] );
    s4 = ( B3 ) * ( vect_acc_old3[idx][dir] );
    s5 = ( B4 ) * ( vect_acc_old4[idx][dir] );
    s7 = ( A1 ) * ( vect_acc_old_filt[idx][dir] );
    s8 = ( A2 ) * ( vect_acc_old2_filt[idx][dir] );
    s9 = ( A3 ) * ( vect_acc_old3_filt[idx][dir] );
    s10 = ( A4 ) * ( vect_acc_old4_filt[idx][dir] );

    vect_acc_filt[idx][dir] = ( s1 + s2 + s3 + s4 + s5 -s7 -s8 -s9 - s10 ); 

    return ( vect_acc_filt[idx][dir]); 
}

/*******************************************************************************
* Function Name: Signal Filtering
*********************************************************************************/
    // create a name for the file output
    // create and open the .csv file
void SignalFiltering(uint8_t i){

    
    vect_filt[i][0] = BandPassFilter(acc_values_mod[i][0], i, 0);
	vect_filt[i][1] = BandPassFilter(acc_values_mod[i][1], i, 1);
	vect_filt[i][2] = BandPassFilter(acc_values_mod[i][2], i, 2);
		
	matrice_completa[3*i+0] = vect_filt[i][0];
	matrice_completa[3*i+1] = vect_filt[i][1];
	matrice_completa[3*i+2] = vect_filt[i][2]; 

	/*matrice_completa[3*i+0] = acc_values_mod[i][0];
	matrice_completa[3*i+1] = acc_values_mod[i][1];
	matrice_completa[3*i+2] = acc_values_mod[i][2];*/
}

/*******************************************************************************
* Function Name: Single Axis Mapping
*********************************************************************************/

void SingleAxisMapping (uint8_t i){
        
    vect_acc_filt_sum[i] = fabs(matrice_completa[3*i+0]) + fabs(matrice_completa[3*i+1]) + fabs(matrice_completa[3*i+2]);
    
	/*if (vect_acc_filt_sum[i] < SOGLIA_PARAM)
    {
        vect_acc_filt_sum[i] = 0.0;
    } 
    else 
    {
        vect_acc_filt_sum[i] = vect_acc_filt_sum[i] - SOGLIA_PARAM;
    }*/
}

/*******************************************************************************
* Function Name: Generate PWM Value
*********************************************************************************/

int GeneratePWMValue(float acc_filt_sum){ 
    int pwm_hapt;
    pwm_hapt = (int)((-SCALA * acc_filt_sum));
	if (pwm_hapt > -5)
	pwm_hapt=0;
	if (pwm_hapt < -28)
	pwm_hapt=-35;
    return pwm_hapt;
}

/*******************************************************************************
* Function Name: Actuators input computation
*********************************************************************************/

void ActuatorsInputComputation (uint8_t i){
    input_act_sent[i-1]= GeneratePWMValue(vect_acc_filt_sum[i]); //vect_acc_filt_sum -> NO STANDARD SCORE -- signal -> STANDARD SCORE

	//cout << "input " << input_act_sent[0]   << " imu: " << i <<  endl;// Further signal manipulation 
}


/*******************************************************************************
* Function Name: Air Chambers Control
*********************************************************************************/

/*
void air_chambers_control(uint8_t i) {

    int16 curr_diff;
    int32 pressure_reference;
    int32 err_pressure, pressure_value;
    int32 valve_command;
    int16 x_value;

    // Use pressure and residual current read from the SoftHand
    
    curr_diff = (int16)commReadResCurrFromSH();

    // Compute pressure reference from residual current

    x_value = curr_diff - 50.0; //mA errore del motore (soglia)
    if (x_value < 0)
        x_value = 0;
    
    pressure_reference = (int32)((int32)(-30.0*x_value*x_value + 55.0*c_mem.FB.max_residual_current*x_value)/(c_mem.FB.max_residual_current*c_mem.FB.max_residual_current)); // da corrente in kPa
    if (pressure_reference < 0)
        pressure_reference = 0;
    if (pressure_reference > c_mem.FB.maximum_pressure_kPa) // da 15 a 40kPa (25kPa)max errore max
        pressure_reference = c_mem.FB.maximum_pressure_kPa;
    
    pressure_value = (int32)g_adc_meas.pressure;
    err_pressure = pressure_reference - pressure_value; //25-qualcosa      // error in kPa

// vector  press in 0 has a pressure value and in 1 has the valve value) 
	pressure_reference =press(0); 


	 if (pressure_reference < 0)
        pressure_reference = 0;
    if (pressure_reference > maximum_pressure_kPa)
        pressure_reference = maximum_pressure_kPa;

    if (press(1) <= 0){
        //i.e the hand is opening
        valve_command = 0;  //valve open: air passes
    }
    else {
        //i.e the hand is closing, so valve should stay closed independently from the pressure error
        //if err_pressure greater than 0, it means pressure should increase, so valve should stay closed
        //if err_pressure==0, it means you reached the right pressure, so valve should stay closed
        valve_command = 1;  //3.6V (5V - 2 diodes) - valve close: air doesn't pass
    }


     // err_pressure = pressure_reference - pressure_value;   
    // Pump control 
   
    Pump_refNew = (int32)(c_mem.FB.prop_err_fb_gain*err_pressure); //gain 1 

    //c_mem.FB.prop_err_fb_gain default 1.0 gain since, max err_pressure is 25 and pwm range is approx. 25 ticks [45=2V,70=3V]
    
    // Limit output voltage
    if (Pump_refNew > 80) // 80 (3.5V) 80% of 4.3V (5V - 1 diode)
        Pump_refNew = 80; // 80
    if (Pump_refNew < 20)
        Pump_refNew = 0;
        
    VALVE_Write(valve_command);
    
    
    // Drive slave with reference generated on second motor index
    // Use second motor structures and parameters, only to generate position reference not for PID control
    // IMPORTANT: configure second motor parameters with proper slave parameters
    // motor_control_generic(slave_motor_idx);

}
*/

int main(int argc, char **argv)
{

	//int num_measure = 1956;

	const int n_imu_ = 2;

	int pump; 


	ros::init(argc, argv, "pulse_send"); // Initiate new ROS node named "act_send_ss"

	// NodeHandle is the main access point to communications with the ROS system. The first NodeHandle constructed will fully initialize this node, and the last NodeHandle destructed will close down the node.
	ros::NodeHandle node;

	ros::Subscriber acc_sub = node.subscribe("acc", 1, readCallback);

	ros::Publisher act_pub = node.advertise<std_msgs::Int16>("act", 1, true);

	std_msgs::Int16 act;

	ros::Rate loop_rate(1000); // Desired rate to turn in Hz

	// Device preparation
	cout << "\n\t Pulse Device\n";
	cout << "\n\n------------------------------------------------------------------------\n";
	// Opening the port the device is connected to
	cout << "\n\n Opening COM ports to communicate with the Actuators.";

	const char *port_act = (char *)"/dev/ttyUSB0";

	openRS485(&comm_settings_act, port_act);

	commActivate(&comm_settings_act, ACT_ID_RM, 1);
	// ros::Duration(0.05).sleep();
	// float acc_values[n_imu_][3];
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
	

	return 0;
}

