#include <allegro.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define PI 3.141592654
#define G0 9.81				// gravity acceleration


/********** structure for tasks **********/

struct task_par
{

	int arg;				// task argument
	long wcet;				// in microseconds
	int period; 			// in milliseconds
	int deadline;			// relative (ms)
	int priority;			// in [0.99]
	int dmiss;				// number of misses
	struct timespec at;		// next activity time
	struct timespec dl;		// absolute time
};

/********** structure for trajectory **********/

struct ball
{
	int c;					// color
	float r;				// radius [m]
	float x;				// x coordinate [m]
	float y;				// y coordinate [m]
	float vx;				// x velocity [m/s]
	float vy;				// y velocity [m/s]
	float dt;
};


/*********** functions for tasks ***********/

// timeCopy
void timeCopy(struct timespec *td, struct timespec ts);

// timeAddMsim
void timeAddMs(struct timespec *t, int ms);

// timeCmp
int timeCmp(struct timespec t1, struct timespec t2);

// setPeriod
void setPeriod(struct task_par *tp);

// waitForPeriod
void waitForPeriod(struct task_par *tp);

// deadlineMiss
int deadlineMiss(struct task_par *tp);


/*************** math function **************/

// multiplyMxM
void multiplyMxM(float X[4][4], float Y[4][4], float Z[4][4]);

// multiplyMxV
void multiplyMxV(float X[4][4], float Y[4][1], float Z[4][1]);

// transpMatrix
void transpMatrix(float X[4][4], float Z[4][4]);

// invMatrix
void invMatrix(float X[4][4], float Z[4][4]);

// determinant
float determinant(float a[4][4], float k);

// cofactor
void cofactor(float a[4][4], float k, float b[4][4]);

// gaussRand
float gaussRand();












