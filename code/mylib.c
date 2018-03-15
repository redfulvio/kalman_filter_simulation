#include "mylib.h"

/**************************************************************************************/
/*********************************** TIME COPY ****************************************/
/**************************************************************************************/

void timeCopy(struct timespec *td, struct timespec ts)
{
	//	It copies a source time variable ts in a
	//	destination variable pointed by td

	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

/**************************************************************************************/
/********************************** TIME ADD MS ***************************************/
/**************************************************************************************/

void timeAddMs(struct timespec *t, int ms)
{
	//	It adds a value ms expressed in milliseconds to
	//	the time variable pointed by t

	t->tv_sec += ms/1000;
	t->tv_nsec += (ms%1000)*1000000;

	if(t->tv_nsec > 1000000000)
	{
		t->tv_nsec -= 1000000000;
		t->tv_sec += 1;
	}
}

/**************************************************************************************/
/********************************** TIME COMPARE **************************************/
/**************************************************************************************/

int timeCmp(struct timespec t1, struct timespec t2)
{
	//	It compares two time variables t1 and t2 and
	//	returns 0 if they are equal, 1 if t1 > t2, ‐1 if t1 < t2

	if(t1.tv_sec > t2.tv_sec)
		return 1;
	if(t1.tv_sec < t2.tv_sec)
		return -1;
	if(t1.tv_nsec > t2.tv_nsec)
		return 1;
	if(t1.tv_nsec < t2.tv_nsec)
		return -1;

	return 0;
}

/**************************************************************************************/
/********************************** SET PERIOD ****************************************/
/**************************************************************************************/

void setPeriod(struct task_par *tp)
{
	//	It reads the current time and computes the next activation
	//	time and the absolute deadline of the task

	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	timeCopy(&(tp->at),t);
	timeCopy(&(tp->dl),t);
	timeAddMs(&(tp->at), tp->period);
	timeAddMs(&(tp->dl), tp->period);
}

/**************************************************************************************/
/********************************** WAIT FOR PERIOD ***********************************/
/**************************************************************************************/

void waitForPeriod(struct task_par *tp)
{
	//	Suspends the calling thread until the next activation and,
	//	when awaken, updates activation time and deadline.
	
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);

	timeAddMs(&(tp->at), tp->period);
	timeAddMs(&(tp->dl), tp->period);
}

/**************************************************************************************/
/********************************** DEADLINE MISS *************************************/
/**************************************************************************************/

int deadlineMiss(struct task_par *tp)
{
	//	If the thread is still in execution when re‐activated, it
	//	increments the value of dmiss and returns 1, otherwise
	//	returns 0
	
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);

	if(timeCmp(now, tp->dl) > 0)
	{
		tp-> dmiss++;
		return 1;
	}

	return 0;
}

/***************************************************************************************/
/********************************** MULTIPLTY MxM **************************************/
/***************************************************************************************/

void multiplyMxM(float X[4][4], float Y[4][4], float Z[4][4])
{
	// multiplication between matrices
	// the result is stored in Z

	int i, j, k;
	float tmp[4][4];
	float sum = 0;

	// initialiasing tmp matrix
	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			tmp[i][j] = 0;
		}
	}

	// multiply matrices
	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			for (k=0; k<4; k++)
			{
				sum = sum + X[i][k] * Y[k][j];
			}
			tmp[i][j] = sum;
			sum = 0;
		}
	}

	//load in destination matrix
	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			Z[i][j] = tmp[i][j];
		}
	}
}

/**************************************************************************************/
/********************************** MULTIPLY MxV **************************************/
/**************************************************************************************/

void multiplyMxV(float X[4][4], float Y[4][1], float Z[4][1])
{
	// multiplication between a matrix and a vector
	// the result is stored in Z

	int i, j, k;
	float tmp[4][1];
	float sum = 0;

	// initialiasing tmp vector
	for (i=0; i<4; i++)
		tmp[i][0] = 0;

	// multiply matrices
	for (i=0; i<4; i++)
	{
		for (j=0; j<1; j++)
		{
			for (k=0; k<4; k++)
			{
				sum = sum + X[i][k] * Y[k][j];
			}
			tmp[i][0] = sum;
			sum = 0;
		}
	}

	//load in destination vector
	for (i=0; i<4; i++)
		Z[i][0] = tmp[i][0];
}

/**************************************************************************************/
/********************************** TRANSP MATRIX *************************************/
/**************************************************************************************/

void transpMatrix(float X[4][4], float Z[4][4])
{
	// transposition of a matrix
	// the result is stored in Z

	int i, j;

	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			Z[i][j] = X[j][i];
		}
	}
}

/**************************************************************************************/
/*********************************** INV MATRIX ***************************************/
/**************************************************************************************/

void invMatrix(float X[4][4], float Z[4][4])
{
	// inverse of a matrix
	// the result is stored in Z

	int i, j;
	float tmp[4][4];

	float det;
	det = determinant(X,4);

	cofactor(X,4,tmp);

	transpMatrix(tmp, tmp);

	for (i=0; i<4; i++)
	{
		for (j=0; j<4; j++)
		{
			Z[i][j] = tmp[i][j] / det;
		}
	}
	
}

/**************************************************************************************/
/*********************************** DETERMINANT **************************************/
/**************************************************************************************/

float determinant(float a[4][4],float k)
{
	// determinant of a matrix
	// the result is returned

	  float s=1,det=0,b[4][4];
	  int i,j,m,n,c;

	  if (k==1)
	  {
	  	return (a[0][0]);
	  }
	  else
	  {
	  	det=0;
	  	for (c=0; c<k; c++)
	  	{
	  		m=0;
	  		n=0;

	  		for (i=0; i<k; i++)
	  		{
	  			for (j=0; j<k; j++)
	  			{
	  				b[i][j]=0;

	  				if (i != 0 && j != c)
	  				{
	  					b[m][n]=a[i][j];

	  					if (n<(k-2))
	  						n++;
	  					else
	  					{
	  						n=0;
	  						m++;
	  					}
	  				}
	  			}
	  		}
	  		det = det + s * (a[0][c] * determinant(b,k-1));
	  		s=-1 * s;
	  	}
	  }

	  return (det);
	}

/**************************************************************************************/
/************************************ COFACTOR ****************************************/
/**************************************************************************************/

void cofactor(float num[4][4],float f, float fac[4][4])
{  
	// cofactor matrix
	// the result is stored in fac

	float b[4][4];
	//fac[4][4];
	int p,q,m,n,i,j;
	
	for (q=0; q<f; q++)
	{
		for (p=0; p<f; p++)
			{
				m=0;
				n=0;
				for (i=0; i<f; i++)
				{
					for (j=0; j<f; j++)
						{
							if (i != q && j != p)
								{
									b[m][n] = num[i][j];
									
									if (n<(f-2))
										n++;
									else
									{
										n=0;
										m++;
									}
								}
						}
				}

				fac[q][p] = pow(-1,q+p) * determinant(b,f-1);
			}
	}
	//transpose(num,fac,f);	
}

/**************************************************************************************/
/************************************ GAUSS RAND **************************************/
/**************************************************************************************/

float gaussRand()
{
	// it generates random numbers with a Gaussian distribution
	// method described by Abramowitz and Stegun

	static float U, V;
	static int phase = 0;
	float Z;

	if(phase == 0) {
		U = (rand() + 1.) / (RAND_MAX + 2.);
		V = rand() / (RAND_MAX + 1.);
		Z = sqrt(-1 * log(U)) * sin(2 * PI * V);

		//printf("!!!\t%f\t%f\t%f\t\n", U,V,Z);
	} else
		Z = sqrt(-1 * log(U)) * cos(2 * PI * V);

	phase = 1 - phase;

	return Z;
}