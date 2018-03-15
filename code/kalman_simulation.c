#define _GNU_SOURCE

#define XWIN 1500                // window x resolution
#define YWIN 1000                // window y resolution

#define WBITMAP 1278             // bitmap width
#define HBITMAP 328              // bitmap height
#define XDEST 111                // bitmap x destination point
#define YDEST 111                // bitmap y destination point

#define NT 9                     // number of threads

/* simulation area limits */
#define BOUNDX1 120              // lower limit x
#define BOUNDX2 1380             // upper limit x
#define BOUNDY1 120              // lower limit y
#define BOUNDY2 430              // upper limit y

#define LEN 30                   // string length
#define MAX_BYTES_PER_CHAR 3     // max bites per char

#define NX    0                  // x noise position into the sliders 
#define NY    1                  // y noise position into the sliders 
#define FOT   2                  // fade-out time position into the sliders 
#define PT    3                  // prediction time position into the sliders 

#define POS_A 3                  // position of the 
#define POS_B 20                 // first element
#define POS_H 37                 // of each matrix
#define POS_Q 54                 // in the dialog
#define POS_R 71

#define POS_NX  88               // position of sliders
#define POS_NY  90               // in the dialog
#define POS_FOT 92
#define POS_PT  94

#define POS_STRING 95            // string postion in the dialog

#include "mylib.h"


/* init functions */
void initGUI();
void initTasks();
void initMatrices();

/* customised dialog objects */
int my_edit_proc(int msg, DIALOG *d, int c);
int my_slider_proc(int msg, DIALOG *d, int c);
int my_button_proc(int msg, DIALOG *d, int c);

/* functions for updating sliders and matrices values*/
int updateMatrixElement(void);
void updateNoise(void *dp3, int val);
void updatePredictionTime(void *dp3, int val);
void updateFadeoutTime(void *dp3, int val);

/* buttons functions */
void startSim(void);
void restartSim();
void pauseSim();
void restoreDefaultValues();
void showPrediction();
void fromM2T();
void fromT2M();
int info();
int quit();

/* some tasks */
void* restoreBackground();
void* drawPrediction();
void* takePosition();
void* drawPoint();
void* trajectory();
void* drawTraj();

DIALOG *myd;                     // a gloabl DIALOG object
BITMAP *buffer_black;            // bitmap for background 
struct ball myb;                 // structure for trajectory

struct sched_param mypar;        // parameters structure
struct task_par tp[NT];          // task structure
pthread_t tid[NT];               // task id
pthread_attr_t attr[NT];         // task attr

sem_t sem_mouse;                 // semaphore for mouse position
sem_t sem_traj;                  // semaphore for trajectory


/***** matrices to display *****/

char Amatrix [4][4][(LEN + 1) * MAX_BYTES_PER_CHAR] = {
   {"1.0", "0.0","0.2","0.0"},
   {"0.0", "1.0","0.0","0.2"},
   {"0.0", "0.0","1.0","0.0"},
   {"0.0", "0.0","0.0","1.0"},
};

char Bmatrix [4][4][(LEN + 1) * MAX_BYTES_PER_CHAR] = {
   {"1.0", "0.0","0.0","0.0"},
   {"0.0", "1.0","0.0","0.0"},
   {"0.0", "0.0","1.0","0.0"},
   {"0.0", "0.0","0.0","1.0"},
};

char Hmatrix [4][4][(LEN + 1) * MAX_BYTES_PER_CHAR] = {
   {"1.0", "0.0","1.0","0.0"},
   {"0.0", "1.0","0.0","1.0"},
   {"0.0", "0.0","0.0","0.0"},
   {"0.0", "0.0","0.0","0.0"},
};

char Qmatrix [4][4][(LEN + 1) * MAX_BYTES_PER_CHAR] = {
   {"0.0", "0.0","0.0","0.0"},
   {"0.0", "0.0","0.0","0.0"},
   {"0.0", "0.0","0.1","0.0"},
   {"0.0", "0.0","0.0","0.1"},
};

char Rmatrix [4][4][(LEN + 1) * MAX_BYTES_PER_CHAR] = {
   {"0.1", "0.0","0.0","0.0"},
   {"0.0", "0.1","0.0","0.0"},
   {"0.0", "0.0","0.1","0.0"},
   {"0.0", "0.0","0.0","0.1"},
};

/* matrices for algorithms */
float AM[4][4];
float BM[4][4];
float HM[4][4];
float QM[4][4];
float RM[4][4];

/* matrices for reinitialisation */
float AM_start[4][4];
float BM_start[4][4];
float HM_start[4][4];
float QM_start[4][4];
float RM_start[4][4];

/* state covariance matrix */
float P[4][4] = {
   {0,    0,    0,    0},
   {0,    0,    0,    0},
   {0,    0,    0,    0},
   {0,    0,    0,    0},
};

/* Kalman gain matrix */
float L[4][4] = {
   {1,    0,    0,    0},
   {0,    1,    0,    0},
   {0,    0,    1,    0},
   {0,    0,    0,    1},
};

/* system state */
float x[4][1] = {
   {600},
   {300},
   {0},
   {0},
};

/* predicted state */
float xpred[4][1] = {
   {600},
   {300},
   {0},
   {0},
};

/* measurement */
float y[4][1] = {
   {600},
   {300},
   {0},
   {0},
};

float old_x = 0;                 // old x mouse position
float old_y = 0;                 // old y mouse position
int start_x = 600;               // initial x point
int start_y = 300;               // initial y point
int end = 0;                     // for threads loop
int period_bg = 50;              // period of task restoreBackground

char the_string[(LEN + 1) * MAX_BYTES_PER_CHAR] = "42";    // for matrix values
char string_nx [(LEN + 1) * MAX_BYTES_PER_CHAR] = "5";      // for slider x value
char string_ny [(LEN + 1) * MAX_BYTES_PER_CHAR] = "5";      // for slider y value
char string_fot [(LEN + 1) * MAX_BYTES_PER_CHAR] = "50";    // for slider fot value
char string_pt [(LEN + 1) * MAX_BYTES_PER_CHAR] = "10";     // for slider pt value

float *value;     // to update matrix value

/* sliders */
int sliders[4] =
{
   5,             // x noise
   5,             // y noise
   50,            // fade-out time
   10,            // prediction time
};

int noise_x;      // variable for updating
int noise_y;      // variable for updating
int pred_time;    // variable for updating
int fot_time;     // variable for updating

int change = 42;  // flag for buttons fromM2T and fromT2M
int draw = 42;    // flag for task showPrediction

/******************************************************************************************************************************************************/
/********************************************************** THE_DIALOG ********************************************************************************/
/******************************************************************************************************************************************************/

DIALOG the_dialog[] =
{
   /* (dialog proc)     (x)   (y)  (w)  (h) (fg) (bg) (key)  (flags)  (d1) (d2) (dp) (dp2) (dp3) */
   
   /* this element just clears the screen, therefore it should come before the others */
   { d_clear_proc,        0,   0,    0,   0,   0,  8,    0,      0,       0,   0,    NULL, NULL, NULL  },

   /* title */
   { d_text_proc,        600, 60, 30, 30, 14, 8, 0, 0, 0, 0, "Kalman Filter Simulation", NULL, NULL},

   /********************************************************* A MATRIX *********************************************************************************/

   { d_text_proc,       200, 520, 30, 30, 14, 8, 0, 0, 0, 0, "A Matrix", NULL, NULL},
   { my_edit_proc,       150, 550,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[0][0], (void*)updateMatrixElement, &AM[0][0]  },
   { my_edit_proc,       200, 550,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[0][1], (void*)updateMatrixElement, &AM[0][1]  },
   { my_edit_proc,       250, 550,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[0][2], (void*)updateMatrixElement, &AM[0][2]  }, 
   { my_edit_proc,       300, 550,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[0][3], (void*)updateMatrixElement, &AM[0][3]  },

   { my_edit_proc,       150, 600,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[1][0], (void*)updateMatrixElement, &AM[1][0]  },
   { my_edit_proc,       200, 600,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[1][1], (void*)updateMatrixElement, &AM[1][1]  },
   { my_edit_proc,       250, 600,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[1][2], (void*)updateMatrixElement, &AM[1][2]  },
   { my_edit_proc,       300, 600,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[1][3], (void*)updateMatrixElement, &AM[1][3]  },

   { my_edit_proc,       150, 650,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[2][0], (void*)updateMatrixElement, &AM[2][0]  },
   { my_edit_proc,       200, 650,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[2][1], (void*)updateMatrixElement, &AM[2][1]  },
   { my_edit_proc,       250, 650,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[2][2], (void*)updateMatrixElement, &AM[2][2]  },
   { my_edit_proc,       300, 650,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[2][3], (void*)updateMatrixElement, &AM[2][3]  },

   { my_edit_proc,       150, 700,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[3][0], (void*)updateMatrixElement, &AM[3][0]  },
   { my_edit_proc,       200, 700,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[3][1], (void*)updateMatrixElement, &AM[3][1]  },
   { my_edit_proc,       250, 700,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[3][2], (void*)updateMatrixElement, &AM[3][2]  },
   { my_edit_proc,       300, 700,  40,  40,   0,  8,    0,      0,     LEN,   0,    Amatrix[3][3], (void*)updateMatrixElement, &AM[3][3]  },

   /********************************************************* B MATRIX *********************************************************************************/
   
   { d_text_proc,       450, 520, 30, 30, 14, 8, 0, 0, 0, 0, "B Matrix", NULL, NULL},
   { my_edit_proc,       400, 550,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[0][0], (void*)updateMatrixElement, &BM[0][0]  },
   { my_edit_proc,       450, 550,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[0][1], (void*)updateMatrixElement, &BM[0][1]  },
   { my_edit_proc,       500, 550,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[0][2], (void*)updateMatrixElement, &BM[0][2]  },
   { my_edit_proc,       550, 550,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[0][3], (void*)updateMatrixElement, &BM[0][3]  },

   { my_edit_proc,       400, 600,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[1][0], (void*)updateMatrixElement, &BM[1][0]  },
   { my_edit_proc,       450, 600,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[1][1], (void*)updateMatrixElement, &BM[1][1]  },
   { my_edit_proc,       500, 600,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[1][2], (void*)updateMatrixElement, &BM[1][2]  },
   { my_edit_proc,       550, 600,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[1][3], (void*)updateMatrixElement, &BM[1][3]  },

   { my_edit_proc,       400, 650,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[2][0], (void*)updateMatrixElement, &BM[2][0]  },
   { my_edit_proc,       450, 650,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[2][1], (void*)updateMatrixElement, &BM[2][1]  },
   { my_edit_proc,       500, 650,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[2][2], (void*)updateMatrixElement, &BM[2][2]  },
   { my_edit_proc,       550, 650,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[2][3], (void*)updateMatrixElement, &BM[2][3]  },

   { my_edit_proc,       400, 700,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[3][0], (void*)updateMatrixElement, &BM[3][0]  },
   { my_edit_proc,       450, 700,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[3][1], (void*)updateMatrixElement, &BM[3][1]  },
   { my_edit_proc,       500, 700,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[3][2], (void*)updateMatrixElement, &BM[3][2]  },
   { my_edit_proc,       550, 700,  40,  30,   0,  8,    0,      0,     LEN,   0,    Bmatrix[3][3], (void*)updateMatrixElement, &BM[3][3]  },

   /********************************************************* H MATRIX *********************************************************************************/
   
   { d_text_proc,       100, 760, 30, 30, 14, 8, 0, 0, 0, 0, "H Matrix", NULL, NULL},
   { my_edit_proc,        50, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[0][0], (void*)updateMatrixElement, &HM[0][0]  },
   { my_edit_proc,       100, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[0][1], (void*)updateMatrixElement, &HM[0][1]  },
   { my_edit_proc,       150, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[0][2], (void*)updateMatrixElement, &HM[0][2]  },
   { my_edit_proc,       200, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[0][3], (void*)updateMatrixElement, &HM[0][3]  },

   { my_edit_proc,        50, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[1][0], (void*)updateMatrixElement, &HM[1][0]  },
   { my_edit_proc,       100, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[1][1], (void*)updateMatrixElement, &HM[1][1]  },
   { my_edit_proc,       150, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[1][2], (void*)updateMatrixElement, &HM[1][2]  },
   { my_edit_proc,       200, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[1][3], (void*)updateMatrixElement, &HM[1][3]  },

   { my_edit_proc,        50, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[2][0], (void*)updateMatrixElement, &HM[2][0]  },
   { my_edit_proc,       100, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[2][1], (void*)updateMatrixElement, &HM[2][1]  },
   { my_edit_proc,       150, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[2][2], (void*)updateMatrixElement, &HM[2][2]  },
   { my_edit_proc,       200, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[2][3], (void*)updateMatrixElement, &HM[2][3]  },

   { my_edit_proc,        50, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[3][0], (void*)updateMatrixElement, &HM[3][0]  },
   { my_edit_proc,       100, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[3][1], (void*)updateMatrixElement, &HM[3][1]  },
   { my_edit_proc,       150, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[3][2], (void*)updateMatrixElement, &HM[3][2]  },
   { my_edit_proc,       200, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Hmatrix[3][3], (void*)updateMatrixElement, &HM[3][3]  },

   /********************************************************* Q MATRIX *********************************************************************************/
   
   { d_text_proc,       350, 760, 30, 30, 14, 8, 0, 0, 0, 0, "Q Matrix", NULL, NULL},
   { my_edit_proc,       300, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[0][0], (void*)updateMatrixElement, &QM[0][0]  },
   { my_edit_proc,       350, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[0][1], (void*)updateMatrixElement, &QM[0][1]  },
   { my_edit_proc,       400, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[0][2], (void*)updateMatrixElement, &QM[0][2]  },
   { my_edit_proc,       450, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[0][3], (void*)updateMatrixElement, &QM[0][3]  },

   { my_edit_proc,       300, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[1][0], (void*)updateMatrixElement, &QM[1][0]  },
   { my_edit_proc,       350, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[1][1], (void*)updateMatrixElement, &QM[1][1]  },
   { my_edit_proc,       400, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[1][2], (void*)updateMatrixElement, &QM[1][2]  },
   { my_edit_proc,       450, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[1][3], (void*)updateMatrixElement, &QM[1][3]  },

   { my_edit_proc,       300, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[2][0], (void*)updateMatrixElement, &QM[2][0]  },
   { my_edit_proc,       350, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[2][1], (void*)updateMatrixElement, &QM[2][1]  },
   { my_edit_proc,       400, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[2][2], (void*)updateMatrixElement, &QM[2][2]  },
   { my_edit_proc,       450, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[2][3], (void*)updateMatrixElement, &QM[2][3]  },

   { my_edit_proc,       300, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[3][0], (void*)updateMatrixElement, &QM[3][0]  },
   { my_edit_proc,       350, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[3][1], (void*)updateMatrixElement, &QM[3][1]  },
   { my_edit_proc,       400, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[3][2], (void*)updateMatrixElement, &QM[3][2]  },
   { my_edit_proc,       450, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Qmatrix[3][3], (void*)updateMatrixElement, &QM[3][3]  },

   /********************************************************* R MATRIX *********************************************************************************/
   
   { d_text_proc,       600, 760, 30, 30, 14, 8, 0, 0, 0, 0, "R Matrix", NULL, NULL},
   { my_edit_proc,       550, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[0][0], (void*)updateMatrixElement, &RM[0][0]  },
   { my_edit_proc,       600, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[0][1], (void*)updateMatrixElement, &RM[0][1]  },
   { my_edit_proc,       650, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[0][2], (void*)updateMatrixElement, &RM[0][2]  },
   { my_edit_proc,       700, 800,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[0][3], (void*)updateMatrixElement, &RM[0][3]  },

   { my_edit_proc,       550, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[1][0], (void*)updateMatrixElement, &RM[1][0]  },
   { my_edit_proc,       600, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[1][1], (void*)updateMatrixElement, &RM[1][1]  },
   { my_edit_proc,       650, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[1][2], (void*)updateMatrixElement, &RM[1][2]  },
   { my_edit_proc,       700, 850,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[1][3], (void*)updateMatrixElement, &RM[1][3]  },

   { my_edit_proc,       550, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[2][0], (void*)updateMatrixElement, &RM[2][0]  },
   { my_edit_proc,       600, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[2][1], (void*)updateMatrixElement, &RM[2][1]  },
   { my_edit_proc,       650, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[2][2], (void*)updateMatrixElement, &RM[2][2]  },
   { my_edit_proc,       700, 900,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[2][3], (void*)updateMatrixElement, &RM[2][3]  },

   { my_edit_proc,       550, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[3][0], (void*)updateMatrixElement, &RM[3][0]  },
   { my_edit_proc,       600, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[3][1], (void*)updateMatrixElement, &RM[3][1]  },
   { my_edit_proc,       650, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[3][2], (void*)updateMatrixElement, &RM[3][2]  },
   { my_edit_proc,       700, 950,  40,  30,   0,  8,    0,      0,     LEN,   0,    Rmatrix[3][3], (void*)updateMatrixElement, &RM[3][3]  },

   /********************************************************* sliders ***********************************************************************************/

   { d_text_proc,       1200, 550, 30, 30, 14, 8, 0, 0, 0, 0, "X-Axis Random Noise:", NULL, NULL},
   { my_slider_proc,    1178,   580,   256,  20,   3,    8,  0,    0,       20,  0,    NULL,    (void*) updateNoise,  &sliders[NX]  },
   { d_text_proc,       1200, 650, 30, 30, 14, 8, 0, 0, 0, 0, "Y-Axis Random Noise:", NULL, NULL},
   { my_slider_proc,    1178,   680,   256,  20,   3,    8,  0,    0,       20,  0,    NULL,    (void*) updateNoise,  &sliders[NY]  },
   { d_text_proc,       1200, 750, 30, 30, 14, 8, 0, 0, 0, 0, "Fade-Out Time (msec):", NULL, NULL},
   { my_slider_proc,    1178,   780,   256,  20,   3,    8,  0,    0,       100,  0,    NULL,    (void*) updateFadeoutTime,  &sliders[FOT]  },
   { d_text_proc,       1200, 850, 30, 30, 14, 8, 0, 0, 0, 0, "Prediction Amount (sec):", NULL, NULL},
   { my_slider_proc,    1178,   880,   256,  20,   3,    8,  0,    0,       20,  0,    NULL,    (void*) updatePredictionTime,  &sliders[PT] },
   
   { d_edit_proc,       1380, 550, 30, 30, 14, 8, 0, 0, LEN, 0, string_nx, NULL, NULL},
   { d_edit_proc,       1380, 650, 30, 30, 14, 8, 0, 0, LEN, 0, string_ny, NULL, NULL},
   { d_edit_proc,       1380, 750, 30, 30, 14, 8, 0, 0, LEN, 0, string_fot, NULL, NULL},
   { d_edit_proc,       1400, 850, 30, 30, 14, 8, 0, 0, LEN, 0, string_pt, NULL, NULL},

   /**************************************************** simulation window ******************************************************************************/
   { d_box_proc,       100, 100,  1300,  350,   4, 0,    0,      0,     0,   0,    NULL, NULL, NULL  },
   { d_box_proc,       110, 110,  1280,  330,   4, 0,    0,      0,     0,   0,    NULL, NULL, NULL  },

   /********************************************************* buttons  **********************************************************************************/

   { my_button_proc,   700, 550,  100,   20,   2,  255,  0, 0,       0,   0,    "Start",  NULL, (void*) startSim  },
   { my_button_proc,   950, 550,  100,   20,   2,  255,  0, 0,       0,   0,    "Restart", NULL, (void*) restartSim  },
   { my_button_proc,   830, 550,  100,   20,   1,  255,  0, 0,       0,   0,    "Pause", NULL, (void*) pauseSim  },
   { my_button_proc,   870, 610,  200,   20,   6,  255,  0, 0,       0,   0,    "Restore Default Values", NULL, (void*) restoreDefaultValues },
   { my_button_proc,   700, 610,  150,   20,   4,  255,  0, 0,       0,   0,    "Show Prediction", NULL, (void*) showPrediction  },
   { my_button_proc,   900, 670,  150,   20,   4,  255,  0, 0,       0,   0,    "fromM2T", NULL, (void*) fromM2T  },
   { my_button_proc,   700, 670,  150,   20,   3,  255,  0, 0,       0,   0,    "fromT2M", NULL, (void*) fromT2M  },
   { my_button_proc,   850, 820,  150,   20,   1,  255,  0, 0,       0,   0,    "Info", NULL, (void*) info  },
   { my_button_proc,   850, 870,  150,   20,   4,  255,  0, 0,       0,   0,    "Quit", NULL, (void*) quit  },

   { NULL,              0,    0,    0,    0,    0,    0,    0,    0,       0,    0,    NULL,    NULL, NULL }
};


/**************************************************************************************/
/*************************************** QUIT *****************************************/
/**************************************************************************************/

int quit()
{
   if (alert("Do you want quit?", NULL, NULL, "&Yes", "&No", 'y', 'n') == 1)
      return D_CLOSE;
   else
      return D_O_K;
}

/**************************************************************************************/
/*************************************** INFO *****************************************/
/**************************************************************************************/

int info()
{
   alert("To change a matrix element:", "click the value to modify,", "type and press &Enter.", "&Ok!",NULL,'o',0);

   return D_O_K;
}

/**************************************************************************************/
/********************************** MY_BUTTON_PROC ************************************/
/**************************************************************************************/

int my_button_proc(int msg, DIALOG *d, int c)
{
   int ret = d_button_proc(msg, d, c);

   if (msg == MSG_CLICK && d->dp3)
   {
      printf("Button clicked!\n");
      return ((int (*)(void))d->dp3)();
   }
   return ret;
}

/**************************************************************************************/
/********************************** MY_SLIDER_PROC ************************************/
/**************************************************************************************/

int my_slider_proc(int msg, DIALOG *d, int c)
{
   int *nos = (int *)d->dp3;

   switch (msg) 
   {
      case MSG_START:
                     /* initialise the slider position */
                     d->d2 = *nos;
                     noise_x = 5;
                     noise_y = 5;
                     fot_time = 50;
                     pred_time = 10;

      break;

      case MSG_IDLE:
                     /* has the slider position changed? */
                     if (d->d2 != *nos) 
                     {
                        d->d2 = *nos;
                        return ((int (*)(void))d->dp2)(); 
                     }
    break;
   }

   return d_slider_proc(msg, d, c);
}

/**************************************************************************************/
/*********************************** MY_EDIT_PROC *************************************/
/**************************************************************************************/

int my_edit_proc(int msg, DIALOG *d, int c)
{
   int ret = d_edit_proc(msg, d, c);

   if (msg == MSG_CLICK)
      myd = d;

   if (keypressed() && d->dp2)
      return ((int (*)(void))d->dp2)();
   
   return ret;
}

/**************************************************************************************/
/*********************************** MASTER TASK 2 ************************************/
/**************************************************************************************/

void* masterTask2(void* arg)
{
   switch (change)
   {
      case 0:

            // takePosition and drawPoint are cancelled

            pthread_cancel(tid[1]);
            pthread_cancel(tid[2]);

            // trajectory and drawTrajectory are activated

            pthread_create(&tid[7], &attr[7], trajectory, &tp[7]);
            pthread_create(&tid[8], &attr[8], drawTraj, &tp[8]);

            pthread_cancel(tid[6]);
      
      break;

      case 1:

            // trajectory and drawTrajectory are cancelled

            pthread_cancel(tid[7]);
            pthread_cancel(tid[8]);

            // takepPosition and drawPoint are activated

            pthread_create(&tid[1], &attr[1], takePosition, &tp[1]);
            pthread_create(&tid[2], &attr[2], drawPoint, &tp[2]);

            pthread_cancel(tid[6]);

      break;  
   }
   
}

/**************************************************************************************/
/******************************** FROM MOUSE TO TRAJ **********************************/
/**************************************************************************************/

void fromM2T()
{
   // a fixed trajectory is set
   
   change = 0;

   pthread_create(&tid[6], &attr[6], masterTask2, &tp[6]);
   
   return D_O_K;
}

/**************************************************************************************/
/******************************** FROM TRAJ TO MOUSE **********************************/
/**************************************************************************************/

void fromT2M()
{
   // the trajectory is drawn by the user

   change = 1;
   
   pthread_create(&tid[6], &attr[6], masterTask2, &tp[6]);
   
   return D_O_K;
}

/**************************************************************************************/
/********************************** DRAW TRAJECTORY ***********************************/
/**************************************************************************************/

void* drawTraj(void* arg)
{
   // it draws the trajectory generated by task "trajectory"
   
   struct task_par *tp;
   tp = (struct task_par *) arg;

   setPeriod(tp);

   while(!end)
   {
      sem_wait(&sem_traj);
      
      //printf("Drawing trajecotry\n");

      if ((myb.x >= BOUNDX1) && (myb.x <= BOUNDX2) && (myb.y >= BOUNDY1) && (myb.y <= BOUNDY2))
         circlefill(screen, myb.x, myb.y, myb.r, myb.c);

      sem_post(&sem_traj);

      if (deadlineMiss(tp))
         printf("Deadline missed in drawing Trajectory!\n");

      waitForPeriod(tp);
   }
}

/**************************************************************************************/
/************************************ TRAJECTORY **************************************/
/**************************************************************************************/

void* trajectory(void* arg)
{
   // it draws a parabolic trajectory

   struct task_par *tp;
   tp = (struct task_par *) arg;

   int color = makecol(137,202,240);
   myb.c = color;
   myb.r = 3;

   // inizial values
   myb.x = 140;
   myb.y = 300;
   myb.vx = 90;
   myb.vy = 50;
   myb.dt = 0.05;

   setPeriod(tp);

   while(!end)
   {
      sem_wait(&sem_traj);

      //printf("Trajectory\n");

      //myb.vx = 140;
      myb.vy = myb.vy - G0 * myb.dt;
      myb.x = myb.x + myb.vx * myb.dt; 
      myb.y = myb.y + myb.vy * myb.dt - 0.5 * G0 * (myb.dt * myb.dt);

      // load in y, corrupted by noise 
      y[0][0] = myb.x + noise_x  * gaussRand();
      y[1][0] = myb.y + noise_y  * gaussRand();
      y[2][0] = myb.vx + noise_x * 0.1 * gaussRand();
      y[3][0] = myb.vy + noise_y * 0.1 * gaussRand();

      sem_post(&sem_traj);

      if (deadlineMiss(tp))
         printf("Deadline missed in trajectory!\n");

      waitForPeriod(tp);
   }
}

/**************************************************************************************/
/********************************* SHOW PREDICTION0 ***********************************/
/**************************************************************************************/

void showPrediction()
{
   position_mouse(start_x, start_y);

   draw = 0;   

   //pthread_create(&tid[4], &attr[4], drawPrediction, &tp[4]);
   
   return D_O_K;
}

/**************************************************************************************/
/********************************* PAUSE SIMULATION ***********************************/
/**************************************************************************************/

void pauseSim()
{
   pthread_cancel(tid[0]);
   pthread_cancel(tid[3]);
   pthread_cancel(tid[4]);
   pthread_cancel(tid[5]);

   if (change == 0)
   {
      pthread_cancel(tid[7]);
      pthread_cancel(tid[8]);
   }
   else if (change == 1)
   {
      pthread_cancel(tid[1]);
      pthread_cancel(tid[2]);
   }


   return D_O_K;
}

/**************************************************************************************/
/******************************** RESTART SIMULATION **********************************/
/**************************************************************************************/

void restartSim()
{
   restoreDefaultValues();
   
   int i,j;
   int n = 4;

   for (i=0; i<n; i++)
   {
      for (j=0; j<n; j++)
      {
         if (i==j)
            L[i][j] = 1;
         else
            L[i][j] = 0;

         P[i][j] = 0;
      }

  
      x[i][0] = 0;
      xpred[i][0] = 0;
      y[i][0] = 0;
   }
   
   // restore positions

   x[0][0] = start_x;
   x[1][0] = start_y;
   xpred[0][0] = start_x;
   xpred[1][0] = start_y;
   y[0][0] = start_x;
   y[1][0] = start_y;

   startSim();

   return D_O_K;
}

/**************************************************************************************/
/****************************** RESTORE DEFAULT VALUES ********************************/
/**************************************************************************************/

void restoreDefaultValues()
{
   int i,j;
   int n = 4;

   int pos_A = (int) POS_A; 
   int pos_B = (int) POS_B; 
   int pos_H = (int) POS_H; 
   int pos_Q = (int) POS_Q; 
   int pos_R = (int) POS_R; 

   for (i=0; i<n; i++)
   {
      for (j=0; j<n; j++)
      {
         AM[i][j] = AM_start[i][j];
         BM[i][j] = BM_start[i][j];
         HM[i][j] = HM_start[i][j];
         QM[i][j] = QM_start[i][j];
         RM[i][j] = RM_start[i][j];

         sprintf(Amatrix[i][j], "%1.1f", AM_start[i][j]);
         sprintf(Bmatrix[i][j], "%1.1f", BM_start[i][j]);
         sprintf(Hmatrix[i][j], "%1.1f", HM_start[i][j]);
         sprintf(Qmatrix[i][j], "%1.1f", QM_start[i][j]);
         sprintf(Rmatrix[i][j], "%1.1f", RM_start[i][j]);

         object_message(&the_dialog[pos_A], MSG_DRAW, 0);
         object_message(&the_dialog[pos_B], MSG_DRAW, 0);
         object_message(&the_dialog[pos_H], MSG_DRAW, 0);
         object_message(&the_dialog[pos_Q], MSG_DRAW, 0);
         object_message(&the_dialog[pos_R], MSG_DRAW, 0);

         pos_A ++;
         pos_B ++;
         pos_H ++;
         pos_Q ++;
         pos_R ++;     
      }
   }

   draw = 42;  // restore flag for showPrediction

   /* restore sliders default values */

   //default values
   sliders[0] = 5;
   sliders[1] = 5;
   sliders[2] = 50;
   sliders[3] = 10;

   the_dialog[POS_NX].d2 = 5;
   the_dialog[POS_NY].d2 = 5;
   the_dialog[POS_FOT].d2 = 50;
   the_dialog[POS_PT].d2 = 10;

   //the_dialog[POS_STRING].dp = "5";
   sprintf(string_nx,"%d", sliders[NX]);
   sprintf(string_ny,"%d", sliders[NY]);
   sprintf(string_fot,"%d", sliders[FOT]);
   sprintf(string_pt,"%d", sliders[PT]);

   for (i=0; i<3; i++)
      object_message(&the_dialog[POS_STRING+i], MSG_DRAW, 0);

   object_message(&the_dialog[POS_NX], MSG_DRAW, 0);
   object_message(&the_dialog[POS_NY], MSG_DRAW, 0);
   object_message(&the_dialog[POS_FOT], MSG_DRAW, 0);
   object_message(&the_dialog[POS_PT], MSG_DRAW, 0);

   return D_O_K;
}

/**************************************************************************************/
/****************************** UPDATE FADE-OUT TIME **********************************/
/**************************************************************************************/

void updateFadeoutTime(void *dp3, int val)
{
   int type = ((uintptr_t)dp3 - (uintptr_t)sliders) / sizeof(sliders[0]);

   if (sliders[type] != val) 
   {
      sliders[type] = val;

      fot_time = sliders[FOT];

      printf("MODIFIED SLIDER FADE-OUT\n");
      printf("fot_time\t%d\n", fot_time);

      pthread_cancel(tid[5]);
      tp[5].period = period_bg + fot_time;
      printf("NEW PERIOD\t%d\n", tp[5].period);
      pthread_create(&tid[5], &attr[5], restoreBackground, &tp[5]);

      object_message(&the_dialog[POS_FOT], MSG_DRAW, 0);

      sprintf(string_fot,"%d",fot_time);
      object_message(&the_dialog[POS_STRING+2], MSG_DRAW, 0);
   }

   return D_O_K;
}

/**************************************************************************************/
/*************************** UPDATE PREDICTION TIME ***********************************/
/**************************************************************************************/

void updatePredictionTime(void *dp3, int val)
{
   int type = ((uintptr_t)dp3 - (uintptr_t)sliders) / sizeof(sliders[0]);

   if (sliders[type] != val) 
   {
      sliders[type] = val;
   
      pred_time = sliders[PT] * 5;
      printf("MODIFIED SLIEDER TIMEPRED\n");
      printf("pred_time\t%d\n", pred_time);

      object_message(&the_dialog[POS_PT], MSG_DRAW, 0);

      sprintf(string_pt,"%d", sliders[PT]);
      object_message(&the_dialog[POS_STRING+3], MSG_DRAW, 0);
   }

   return D_O_K;
}

/**************************************************************************************/
/*********************************** UPDATE NOISE *************************************/
/**************************************************************************************/

void updateNoise(void *dp3, int val)
{
   int type = ((uintptr_t)dp3 - (uintptr_t)sliders) / sizeof(sliders[0]);

   if (sliders[type] != val) 
   {
      sliders[type] = val;

      noise_x = sliders[NX];
      noise_y = sliders[NY];

      printf("MODIFIED SLIEDER NOISE\n");
      printf("noise_x\t%d\n", noise_x);
      printf("noise_y\t%d\n", noise_y);

      object_message(&the_dialog[POS_NX], MSG_DRAW, 0);
      object_message(&the_dialog[POS_NY], MSG_DRAW, 0);

      sprintf(string_nx,"%d",noise_x);
      sprintf(string_ny,"%d",noise_y);
      object_message(&the_dialog[POS_STRING], MSG_DRAW, 0);
      object_message(&the_dialog[POS_STRING+1], MSG_DRAW, 0);
   }

   return D_O_K;
}

/**************************************************************************************/
/******************************* UPDATE MATRIX ELEMENT ********************************/
/**************************************************************************************/

int updateMatrixElement(void)
{
   float str;

   if (keypressed())
   {
      value = (float *) myd->dp3;
      //printf("VALUE\t%3.3f\n", *value);

            if ( (key[KEY_0] || key[KEY_1] ||key[KEY_2] || key[KEY_3] || key[KEY_4] || key[KEY_5] || key[KEY_6] ||
               key[KEY_7] || key[KEY_8] || key[KEY_9] || key[KEY_BACKSPACE] || key[KEY_ENTER ]) )
            {    /* click KEY_ENTER to update the new value*/

               strcpy(the_string, myd->dp);
   
               str = atof(the_string);
               //printf("STR HAS VALUE\t%3.3f\n", str);
               *value = str;
               //printf("Nuovo value\t%3.3f\n", *value);
            }
   }

   return D_O_K;
}

/******************************************************************************************/
/**************************************** KALMAN ******************************************/
/******************************************************************************************/

void* Kalman(void* arg)
{
   struct task_par *tp;
   tp = (struct task_par *) arg;

   int i, j;

   setPeriod(tp);

   float Atrasp[4][4], Htrasp[4][4];
   float HP[4][4], HPH[4][4], HPHR[4][4], inn[4][1];
   float I[4][4];
   float tmp[4][4];
   float Hx[4][1];

   for (i=0; i<4; i++)
   {
      for (j=0; j<4; j++)
      {
         if (i == j)
            I[i][j] = 1;
         else
            I[i][j] = 0;
      }
   }

   while(!end)
   {
      //printf("Kalman\n");

      /* necessary for prediction */

      // A'
      transpMatrix(AM,Atrasp);
      // A * P(k-1)
      multiplyMxM(AM,P,P);
      // (A * P(k-1)) * A'
      multiplyMxM(P,Atrasp,P);
      
      /***** PREDICTION *****/

      // x-(k) = A * x(k-1)
      multiplyMxV(AM,x,x);

      // P(k) = A * P(k-1) * A' + Q
      for (i=0; i<4; i++)
      {
         for (j=0; j<4; j++)
         {
            P[i][j] = P[i][j] + QM[i][j];
         }
      }


      /* necessary for correction */
      
      // H * P-(k)
      multiplyMxM(HM,P,HP);
      // H'
      transpMatrix(HM,Htrasp);
      // H * P-(k) * H'
      multiplyMxM(HP,Htrasp,HPH);

      // H * P-(k) * H' + R
      for (i=0; i<4; i++)
      {
         for (j=0; j<4; j++)
         {
            HPHR[i][j] = HPH[i][j] + RM[i][j];
         }
      }

      // (H * P-(k) * H' + R)^-1
      invMatrix(HPHR,HPHR);
      // P-(k) * H'
      multiplyMxM(P,Htrasp,P);
      // H * x-(k)
      multiplyMxV(HM,x,Hx);

      // y(k) - H * x-(k)
      for (i=0; i <4; i++)
         inn[i][0] = y[i][0] - Hx[i][0];  


      /***** CORRECTION *****/

      // L(k) = P-(k) * H' * (H * P-(k) * H' + R)^-1 
      multiplyMxM(P,HPHR,L);
      // L(k) * (y(k) - H * x-(k))
      multiplyMxV(L,inn,inn);

      // x^(k) = x-(k) + L(k) * (y(k) - H * x-(k))
      for (i=0; i<4; i++)
         x[i][0] = x[i][0] + inn[i][0];

      // L(k) * H
      multiplyMxM(L,HM,tmp);
      // I - L(k) * H
      for (i=0; i<4; i++)
      {
         for (j=0; j<4; j++)
         {
            tmp[i][j] = I[i][j] - tmp[i][j];
         }
      }
      // P(k) = (I - L(k) * H) * P-(k)
      multiplyMxM(tmp,P,P);

      if (deadlineMiss(tp))
         printf("Deadline missed in Kalman!\n");

      waitForPeriod(tp);
   }
}

/******************************************************************************************/
/*********************************** DRAW PREDICTION **************************************/
/******************************************************************************************/

void* drawPrediction(void* arg)
{
   struct task_par *tp;
   tp = (struct task_par *) arg;

   int i, t;
   int color1 = makecol(255,0,0);
   int color2 = makecol(26,245,22);
   int radius = 3;

   setPeriod(tp);

   while (!end)
   {  
      //printf("Prediction\n");

      for (i=0; i<4; i++)
         xpred[i][0] = x[i][0];

      for (t=0; t<pred_time; t++)
         multiplyMxV(AM,xpred,xpred);

  while( !(xpred[0][0] >= BOUNDX1) && (xpred[0][0] <= BOUNDX2) && (xpred[1][0] >= BOUNDY1) && (xpred[1][0] <= BOUNDY2) )
      {
         if (xpred[0][0] <= BOUNDX1)
            xpred[0][0] += 5;

         if (xpred[0][0] >= BOUNDX2)
            xpred[0][0] -= 5;

         if (xpred[1][0] <= BOUNDY1)
            xpred[1][0] += 5;

         if (xpred[1][0] >= BOUNDY2)
            xpred[1][0] -= 5;
      }

      if ( (xpred[0][0] >= BOUNDX1) && (xpred[0][0] <= BOUNDX2) && (xpred[0][1] >= BOUNDY1) && (xpred[0][1] <= BOUNDY2) && (draw == 0) )
      {
         circlefill(screen, x[0][0], x[1][0], radius, color2);   
         line(screen, x[0][0], x[1][0], xpred[0][0], xpred[1][0], color1);
      }
   
      if (deadlineMiss(tp))
         printf("Deadline missed in drawPrediction!\n");

      waitForPeriod(tp);
   }
}

/******************************************************************************************/
/********************************** RESTORE BACKGROUND ************************************/
/******************************************************************************************/

void* restoreBackground(void* arg)
{
   struct task_par *tp;
   tp = (struct task_par *) arg;

   setPeriod(tp);

   while(!end)
   {
      //printf("Restore BACKGROUNG\n");

      blit(buffer_black, screen, 0, 0, XDEST, YDEST, WBITMAP, HBITMAP);

      if (deadlineMiss(tp))
         printf("Deadline missed in restore background!\n");

      waitForPeriod(tp);
   }
}

/******************************************************************************************/
/************************************** DRAW POINT ****************************************/
/******************************************************************************************/

void* drawPoint(void* arg)
{
   struct task_par *tp;
   tp = (struct task_par *) arg;

   int color = makecol(137,202,240);
   int radius = 2.5;

   setPeriod(tp);

   while(!end)
   {
      sem_wait(&sem_mouse);
      
      //printf("Drawing\n");

      // draw a point where the mouse focuses
      circlefill(screen, y[0][0], y[1][0], radius, color);

      sem_post(&sem_mouse);

      if (deadlineMiss(tp))
         printf("Deadline missed in drawing point!\n");

      waitForPeriod(tp);
   }
}

/******************************************************************************************/
/************************************ TAKE POSITION ***************************************/
/******************************************************************************************/

void* takePosition(void* arg)
{
   float dt;
   struct timespec tnew, told;

   told.tv_nsec = 0;

   struct task_par *tp;
   tp = (struct task_par *) arg;

   setPeriod(tp);

   while(!end)
   {
      sem_wait(&sem_mouse);

      //printf("Taking\n");

      clock_gettime(CLOCK_MONOTONIC, &tnew);

      if ( (mouse_x >= BOUNDX1) && (mouse_x <= BOUNDX2) && (mouse_y >= BOUNDY1) && (mouse_y <= BOUNDY2) )
      {
         dt = ((float)(tnew.tv_nsec - told.tv_nsec));  // [ms]

         // take the mouse position corrupted by noise
         y[0][0] = mouse_x + noise_x * gaussRand();
         y[1][0] = mouse_y +  noise_y * gaussRand();
         y[2][0] = (y[0][0] - old_x) / dt;
         y[3][0] = (y[1][0] - old_y) / dt;

         told.tv_nsec = tnew.tv_nsec;
         old_x = y[0][0];
         old_y = y[1][0];         
      }

      sem_post(&sem_mouse);
      
      if (deadlineMiss(tp))
         printf("Deadline missed in taking mouse postion!\n");

      waitForPeriod(tp);
   }
}

/******************************************************************************************/
/************************************* MASTER TASK ****************************************/
/******************************************************************************************/

void* masterTask(void* arg)
{
   pthread_create(&tid[1], &attr[1], takePosition, &tp[1]);
   pthread_create(&tid[2], &attr[2], drawPoint, &tp[2]);
   pthread_create(&tid[3], &attr[3], Kalman, &tp[3]);
   pthread_create(&tid[4], &attr[4], drawPrediction, &tp[4]);
   pthread_create(&tid[5], &attr[5], restoreBackground, &tp[5]);
}

/******************************************************************************************/
/*********************************** START SIMULATION *************************************/
/******************************************************************************************/

void startSim(void)
{
   // it moves the mouse inside the simulation window
   position_mouse(start_x, start_y);

   pthread_create(&tid[0], &attr[0], masterTask, &tp[0]);
   
   return D_O_K;
}

/******************************************************************************************/
/************************************** INIT TASKS ****************************************/
/******************************************************************************************/

void initTasks()
{
   // initialise semaphores
   sem_init(&sem_mouse, 0, 1);
   sem_init(&sem_traj, 0, 1);

   int i;
   int td;

   for (i=0; i<NT; i++)
   {
      tp[i].arg = i;
      tp[i].period = 25;
      tp[i].deadline = 150;
      tp[i].priority = 50;
      tp[i].dmiss = 0;      

      pthread_attr_init(&attr[i]);
      pthread_attr_setinheritsched(&attr[i],PTHREAD_EXPLICIT_SCHED);
      pthread_attr_setschedpolicy(&attr[i], SCHED_FIFO);
      pthread_attr_setdetachstate(&attr[i], PTHREAD_CREATE_JOINABLE);
      mypar.sched_priority = tp[i].priority;
      pthread_attr_setschedparam(&attr[i], &mypar);
   }

   // 0 masterTask
   // 1 takePosition
   // 2 drawPoint
   // 3 Kalman
   // 4 drawPrediction
   // 5 restoreBackground
   // 6 masterTask2
   // 7 trajecotry
   // 8 drawTrajectory

   tp[0].priority = 90;
   tp[6].priority = 90;

   tp[5].period = period_bg;
      
   for (i=0; i<NT; i++)
   {
      pthread_join(tid[i], NULL);
      pthread_attr_destroy(&attr[i]);
   }
}

/***************************************************************************************/
/*********************************** INIT MATRICES *************************************/
/***************************************************************************************/

void initMatrices()
{
   //load matrices
   int i, j;
   int n = 4;

   int pos_A = (int) POS_A; 
   int pos_B = (int) POS_B; 
   int pos_H = (int) POS_H; 
   int pos_Q = (int) POS_Q; 
   int pos_R = (int) POS_R; 

   for (i=0; i<n; i++)
   {
      for (j=0; j<n; j++)
      {
         AM[i][j] = atof(the_dialog[pos_A].dp);
         BM[i][j] = atof(the_dialog[pos_B].dp);
         HM[i][j] = atof(the_dialog[pos_H].dp);
         QM[i][j] = atof(the_dialog[pos_Q].dp);
         RM[i][j] = atof(the_dialog[pos_R].dp);

         AM_start[i][j] = atof(the_dialog[pos_A].dp);
         BM_start[i][j] = atof(the_dialog[pos_B].dp);
         HM_start[i][j] = atof(the_dialog[pos_H].dp);
         QM_start[i][j] = atof(the_dialog[pos_Q].dp);
         RM_start[i][j] = atof(the_dialog[pos_R].dp);

         pos_A ++;
         pos_B ++;
         pos_H ++;
         pos_Q ++;
         pos_R ++;      
      }
   }

}

/******************************************************************************************/
/************************************** INIT GUI ******************************************/
/******************************************************************************************/

void initGUI()
{
   if (allegro_init() != 0)
      return 1;

   install_keyboard();
   install_mouse();
   install_timer();

   set_color_depth(8);

   if (set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0) != 0) {
      if (set_gfx_mode(GFX_SAFE, 320, 200, 0, 0) != 0) {
    set_gfx_mode(GFX_TEXT, 0, 0, 0, 0);
    allegro_message("Unable to set any graphic mode\n%s\n", allegro_error);
    return 1;
      }
   }

   int color = makecol(0,0,0);
   buffer_black = create_bitmap(WBITMAP, HBITMAP);
   clear_bitmap(buffer_black);
   clear_to_color(buffer_black,color);
   
   show_mouse(screen);
}

/**************************************************************************************/
/************************************** MAIN ******************************************/
/**************************************************************************************/

int main()
{
   initMatrices();

   initGUI();

   initTasks();

   do_dialog(the_dialog,-1);

   allegro_exit();
}

/**************************************************************************************/
/**************************************************************************************/
/**************************************************************************************/