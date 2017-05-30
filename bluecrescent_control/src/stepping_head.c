#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define QUIT_CHAR 0x03 // ctrl+c

#define lrotate(x) (x=(((0x80 & x) >> 0x7) | (x << 1)))
#define rrotate(x) (x=(((0x01 & x) << 0x7) | (x >> 1)))
#define A(num) (VSET | (step[num].nA & 0x2) | (step[num].A & 0x1)) 
#define B(num) (VSET | (step[num].nB & 0x2) | (step[num].B & 0x1))
#define printstep(num) printf("%x %x %x %x\r\n",0x1 & step[num].A,(0x2&step[num].nA)>>1,0x1&step[num].B,(0x2&step[num].nB)>>1)

// Steps for 1 period loop
#define STEP_N (360/(18/2))
// Degree of 1 step
#define DSTEP 9

// 8830 Register
#define VSET (0x26 << 2)
#define CONTROL 0x0
#define FAULT 0x1

struct termios CookedTermIos;
struct termios RawTermIos;

struct timespec ts;

// addressA=[0x60,0x61] #0xc0
// addressB=[0x62,0x63] #0xc4,0xc8
// #addressA=[0x64,0x65]
// #addressB=[0x66,0x67]

// pulse = 10


// dt = 0.002
// t=0
//h=0
//while True:
//	try:
//    		getch = _Getch()
//    		x = getch()
//    		if (int(x) == 4):
//                    address = addressA
//                    for i in range(0,pulse):
//			t = t + 1
//    		    	cwstep(0)
//                        time.sleep(dt)
//    		        print t
//    		elif (int(x) == 6):
//                    address = addressA
//                    for i in range(0,pulse):
//			t = t - 1
//    		    	ccwstep(0)
//                        time.sleep(dt)
//    		        print t
//    		elif (int(x) == 5):
//			motor_lock(0)
//    		elif (int(x) == 2):
//                    address = addressB
//                    for i in range(0,pulse):
//			h = h - 1
//    		    	cwstep(0)
//                        time.sleep(dt)
//    		        print h
//    		elif (int(x) == 8):
//                    address = addressB
//                    for i in range(0,pulse):
//			h = h + 1
//    		    	ccwstep(0)
//                        time.sleep(dt)
//    		        print h
//    		else:
//			motor_release()
//    			t=0
//	except:
//		motor_release()
//		exit(0)
//


// DRV8830 drvreg Register for phase A,B
typedef struct 
{
 unsigned char CONTROL_A;
 unsigned char CONTROL_B;
 unsigned char FAULT_A;
 unsigned char FAULT_B;
}drv8830reg;

// motorstep step
typedef struct
{
 unsigned char A;
 unsigned char nA;
 unsigned char B;
 unsigned char nB;
}motorstep;

drv8830reg drvreg[4];
//FULL STEP
//motorstep step[4] = {
//	//M0
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M1
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M2
//	{0xcc,(0x33 << 1),0x66,0x33},
//	//M3
//	{0xCC,(0x33 << 1),0x66,0x33}
//};
//HALF STEP
motorstep step[4] = {
	//M0
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M1
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M2
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)},
	//M3
	{0xC1,(0x1C << 1),0x70,(0x07 << 1)}
};

uint32_t stepcnt[4] = {0,0,0,0};
unsigned const char drv8830_addr[2] = {0x60,0x61};
unsigned const char drv8830_addr_M1[2] = {0x62,0x63};
int fd[2];

void motor_release(){
	printf("MOTOR RELEASE!\n");
	wiringPiI2CWriteReg8(fd[0],CONTROL,0x18);
	wiringPiI2CWriteReg8(fd[1],CONTROL,0x18);
}
void motor_lock(uint8_t num){
	printf("MOTOR LOCKED!\n");
	wiringPiI2CWriteReg8(fd[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd[1],CONTROL,B(num));
}
void cwstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd[1],CONTROL,B(num));
}
void ccwstep(uint8_t num){
	rrotate(step[num].A);
	rrotate(step[num].nA);
	rrotate(step[num].B);
	rrotate(step[num].nB);
	wiringPiI2CWriteReg8(fd[0],CONTROL,A(num));
	wiringPiI2CWriteReg8(fd[1],CONTROL,B(num));
}

void fdeg(uint32_t deg, uint8_t num){
	uint32_t loop = deg/DSTEP;
	uint32_t i=0;
	if(i<loop)
		cwstep(num);
}

void sigcatch(int sig){
	printf("received SIGINT.\n");
	motor_release();
	exit(1);
}

int main(void){
	int c;
	ts.tv_sec = 5;
	ts.tv_nsec = 0;

        /* WHO AM I */
        fd[0] = wiringPiI2CSetup(drv8830_addr[0]);
        fd[1] = wiringPiI2CSetup(drv8830_addr[1]);

	// Ctrl+C Abort
	if(SIG_ERR== signal(SIGINT,sigcatch))
		return 1;
	// Key Scan
	tcgetattr(STDIN_FILENO, &CookedTermIos);
	RawTermIos = CookedTermIos;
	cfmakeraw(&RawTermIos);
	tcsetattr(STDIN_FILENO, 0, &RawTermIos);

	//printf( "Motor running during push");
	//clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
	//printf( " 8:up 2:down 5:lock 1:motor1 3:motor2 anynum:release char:exit");
	
	while((c = getchar()) != QUIT_CHAR) {
	//if(isprint(c)) {
		//	putchar(c);
			switch(c){
				case '8':
					cwstep(0);
					printstep(0);
			}

	//	} else {
	//		printf("<%02X>", c);
	//	}
	}


	tcsetattr(STDIN_FILENO, 0, &CookedTermIos);
	return 0;
}
