// CUFF Proprioception Header file

#ifndef PROPRIOCEPTION_FEDE_H
#define PROPRIOCEPTION_FEDE_H
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <assert.h>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include <math.h>
#include <cmath>
#include <getopt.h>
#include <stdarg.h>
#include <iterator>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <utility>
#include <stdexcept>
#include <sstream>

///// Include the qb headers
// #include "imuboard_communications.h"
#include "cp_communications.h"
#include "definitions.h"
#define BUFFER_SIZE 500 ///< Size of buffers that store communication packets

#define SENS_ID 1
#define HAND_ID 16
#define ACT_ID_IT 3
#define ACT_ID_RM 1// prima era 2
#define ACT_ID_PL 4

#define n_imus 1

using namespace std;




     extern float vect_acc[n_imus][3];      // Last Sample (Filter input)
     extern float vect_acc_old[n_imus][3];  // Sample one step back
     extern float vect_acc_old2[n_imus][3]; // Sample two steps back
     extern float vect_acc_old3[n_imus][3]; // Sample three steps back
     extern float vect_acc_old4[n_imus][3];

     extern float vect_acc_filt[n_imus][3];      // Filtered last sample (Filter Output)
     extern float vect_acc_old_filt[n_imus][3];  // Filter output one step back
     extern float vect_acc_old2_filt[n_imus][3]; // Filter output two steps back
     extern float vect_acc_old3_filt[n_imus][3]; // Filter output three steps back
     extern float vect_acc_old4_filt[n_imus][3];


     float vect_acc[n_imus][3];      // Last Sample (Filter input)
     float vect_acc_old[n_imus][3];  // Sample one step back
     float vect_acc_old2[n_imus][3]; // Sample two steps back
     float vect_acc_old3[n_imus][3]; // Sample three steps back
     float vect_acc_old4[n_imus][3];

     float vect_acc_filt[n_imus][3];      // Filtered last sample (Filter Output)
     float vect_acc_old_filt[n_imus][3];  // Filter output one step back
     float vect_acc_old2_filt[n_imus][3]; // Filter output two steps back
     float vect_acc_old3_filt[n_imus][3]; // Filter output three steps back
     float vect_acc_old4_filt[n_imus][3];



//// Function Declarations
int open_port(int port_choice);
void close_all(int);
float mean_arr(float arr[], int size);
float bandpass(float new_value, uint8_t idx, uint8_t dir);
int32_t generate_PWM_value(float acc_filt_sum, int i, float SCALA, int SHIFT);
void int_handler(int sig);
float dev_standard(float arr[], int size, float media);

//// Global variables definition
comm_settings comm_settings_sens, comm_settings_act, comm_settings_t;


// Filter output one step back

#endif
/// Function definitions

void int_handler(int sig)
{
    closeRS485(&comm_settings_sens);
    closeRS485(&comm_settings_act);
    closeRS485(&comm_settings_t);

}

// Function to open the communication port with the CUFF
int open_port()
{
    FILE *file;
    char port_sens[255];

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL)
    {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port_sens);

    fclose(file);

    openRS485(&comm_settings_sens, port_sens, 2000000);

    if (comm_settings_sens.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the sensors serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}

// Closing Function

void close_all(int a)
{
    unsigned char aux_uchar;
    int exit_c;
    printf("\nShutting down application...\n");

    aux_uchar = 0;
    commActivate(&comm_settings_sens, SENS_ID, 0);

    usleep(100000);

    closeRS485(&comm_settings_sens);

    exit_c = 1;

    printf("CLOSED");
}

float mean_arr(float arr[], int size)
{ // size deve essere sempre un +1 rispetto alle dimensioni dell'array perchè cpp parte da 0
    int i;
    float sum = 0.0, avg;

    for (i = 0; i < size; i++)
    {
        sum += arr[i];
    }
    avg = sum / float(size);
    // avg = float(sum/size);

    return avg;
}

float dev_standard(float arr[], int size, float media)
{
    float sigma = 0.0;
    float somma = 0.0;

    for (int i = 0; i < size; i++)
    {
        somma += pow((arr[i] - media), 2);
    }

    // sigma = sqrt(somma) ;
    sigma = sqrt((somma) / float(size - 1));
    // sigma = float(sqrt((somma) / (size-1)));
    return sigma;
}

// Function to implement the bandpass filter in case of float numbers
float bandpass(float new_value, uint8_t idx, uint8_t dir)
{
    // Filter parameters (int case)

    // cout << vect_acc_old4 [idx][dir]<< endl;

    float A0 = (1.0);
    float A1 = (0.19227 / A0);
    float A2 = (1.09197 / A0);
    float A3 = (0.15811 / A0);
    float A4 = (0.72636 / A0);

    float B_0 = (0.06423 / A0);
    float B1 = (0.0 / A0);
    float B2 = (-0.12845 / A0);
    float B3 = (0.0 / A0);
    float B4 = (0.06423 / A0);


    float s1, s2, s3, s4, s5, s6, s7, s8, s9, s10;

    

vect_acc_old4[idx][dir] = vect_acc_old3[idx][dir];
vect_acc_old3[idx][dir] = vect_acc_old2[idx][dir];
vect_acc_old2[idx][dir] = vect_acc_old[idx][dir];
vect_acc_old[idx][dir] = vect_acc[idx][dir];

vect_acc_old4_filt[idx][dir] = vect_acc_old3_filt[idx][dir];
vect_acc_old3_filt[idx][dir] = vect_acc_old2_filt[idx][dir];
vect_acc_old2_filt[idx][dir] = vect_acc_old_filt[idx][dir];
vect_acc_old_filt[idx][dir] = vect_acc_filt[idx][dir];

vect_acc[idx][dir] = new_value;

//cout << " filt"<<  vect_acc_filt[idx][dir] << endl;

s1 = (B_0) * (vect_acc[idx][dir]);
s2 = (B1) * (vect_acc_old[idx][dir]);
s3 = (B2) * (vect_acc_old2[idx][dir]);
s4 = (B3) * (vect_acc_old3[idx][dir]);
s5 = (B4) * (vect_acc_old4[idx][dir]);
s7 = (A1) * (vect_acc_old_filt[idx][dir]);
s8 = (A2) * (vect_acc_old2_filt[idx][dir]);
s9 = (A3) * (vect_acc_old3_filt[idx][dir]);
s10 = (A4) * (vect_acc_old4_filt[idx][dir]);

vect_acc_filt[idx][dir] = (s1 + s2 + s3 + s4 + s5 - s7 - s8 - s9 - s10);



return ( vect_acc_filt[idx][dir]);

/*
    // Filtro 20Hz
    float A0 = (  1.0 );
    float A1 = ( -1.76984 / A0);
    float A2 = (  0.79372 / A0);

    float B_0 =  (  0.89089 / A0 );
    float B1  = ( -1.78178 / A0 );
    float B2  = (  0.89089 / A0 );

*/
/*
    // Filtro 10Hz
    float A0 = (  1.0 );
    float A1 = ( -1.88460 / A0);
    float A2 = (  0.89091 / A0);

    float B_0 =  ( 0.94388 / A0 );
    float B1  = ( -1.88775 / A0 );
    float B2  = (  0.9438 / A0 );
*/

/*
    // Filtro 35Hz
    float A0 = (  1.0 );
    float A1 = ( -1.5998 / A0);
    float A2 = (  0.6675 / A0);

    float B_0 =  ( 0.8168 / A0 );
    float B1  = ( -1.6336 / A0 );
    float B2  = (  0.8168 / A0 );



    float s1, s2, s3, s7, s8;

    static float vect_acc[n_imus][3];            // Last Sample (Filter input)
    static float vect_acc_old[n_imus][3];         // Sample one step back
    static float vect_acc_old2[n_imus][3];        // Sample two steps back


    static float vect_acc_filt[n_imus][3];        // Filtered last sample (Filter Output)
    static float vect_acc_old_filt[n_imus][3];    // Filter output one step back
    static float vect_acc_old2_filt[n_imus][3];   // Filter output two steps back



    vect_acc_old2[idx][dir] = vect_acc_old[idx][dir];
    vect_acc_old[idx][dir] = vect_acc[idx][dir];


    vect_acc_old2_filt[idx][dir] = vect_acc_old_filt[idx][dir];
    vect_acc_old_filt[idx][dir] = vect_acc_filt[idx][dir];

    vect_acc[idx][dir] = new_value;

    s1 = ( B_0 ) * ( vect_acc[idx][dir] );
    s2 = ( B1 ) * ( vect_acc_old[idx][dir] );
    s3 = ( B2 ) * ( vect_acc_old2[idx][dir] );
    s7 = ( A1 ) * ( vect_acc_old_filt[idx][dir] );
    s8 = ( A2 ) * ( vect_acc_old2_filt[idx][dir] );


    vect_acc_filt[idx][dir] = ( s1 + s2 + s3 -s7 -s8);

    return ( vect_acc_filt[idx][dir]);
*/
}

int32_t generate_PWM_value(float acc_filt_sum, int i, float SCALA, int SHIFT)
{

    static int32_t count_time[n_imus];
    static short int reception_state[n_imus];
    int32_t mot_speed;

    /*
    switch( reception_state[i] ){

        case 0:
            if ( acc_filt_sum > 0.0 ){
            count_time[i] = 0;
            reception_state[i] = 1;
            }
        break;

        case 1:
            count_time[i]++;

            if( count_time[i] >= 1){      //10
                count_time[i] = 0;
                acc_filt_sum = 0.0;
                reception_state[i] = 2;
            }
            break;

        case 2:
            count_time[i]++;
            acc_filt_sum = 0.0;
            if ( count_time[i] >= 200 ){  //100
                count_time[i] = 0;
                reception_state[i] = 3;
            }
            break;

        case 3:
            count_time[i]++;
        if ( acc_filt_sum > 0.0 ){
             count_time[i] = 0;
        } else {
            if(count_time[i] >= 300){    //300
                count_time[i] = 0;
                reception_state[i] = 0;
            }
        }
            break;
    }*/

    // cout << "here";

    // mot_speed = (int32_t)(SCALA * acc_filt_sum );

    // if( mot_speed == 0){
    //     mot_speed = 0;
    // }
    // else mot_speed =  mot_speed + SHIFT; // 35 è il valore del pwm per cui i voice
    // coil iniziano a muoversi

    // Non superare mai il valore di 60! E' il valore massimo che si può dare
    // al pwm perchè corrisponde alla tensione massima che posso dare al voice coil
    // con un'alimentazione di 12V
    // if ( mot_speed > 60*24) { // *24
    //    mot_speed = 60*24; //*24
    //}

    static int32_t pwm_hapt;
    //cout << SCALA << endl;
    pwm_hapt = (SCALA * acc_filt_sum);
    return pwm_hapt;
}
