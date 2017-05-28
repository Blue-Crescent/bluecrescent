#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

// IN1
#define INIT_A (0xc1)
// IN2
#define INIT_nA 0x1c << 1
// IN1
#define INIT_B 0x70
// IN2
#define INIT_nB 0x07 << 1
#define VSET (0x26 << 2)

#define lrotate(x) (x=(((0x80 & x) >> 0x7) | (x << 1)))
#define rrotate(x) (x=(((0x01 & x) << 0x7) | (x >> 1)))

// Steps for 1 period loop
#define STEP_N (360/(18/2))
// Degree of 1 step
#define DSTEP 9


addressA=[0x60,0x61] #0xc0
addressB=[0x62,0x63] #0xc4,0xc8
#addressA=[0x64,0x65]
#addressB=[0x66,0x67]
#address[0] = 0x64
#address[1] = 0x65
#VSET = (0x3f<< 2)
VSET = (0x20<< 2)

M0 = 0
M1 = 1
M2 = 2
M3 = 3

address = addressA

pulse = 10

# A,B bit0
# nA,nB are bit1.So, 1bit left shifted

#FULL STEP [A,nA,B,nB] x M0-3
#step =[
#		[0xcc,(0x33 << 1),0x66,0x33], #m0
#		[0xcc,(0x33 << 1),0x66,0x33], #m1
#		[0xcc,(0x33 << 1),0x66,0x33], #m2
#		[0xCC,(0x33 << 1),0x66,0x33]] #M3

#HALF STEP [A,nA,B,nB] x M0-M3
step = [	
		[0xC1,(0x1C << 1),0x70,(0x07 << 1)], #M0
		[0xC1,(0x1C << 1),0x70,(0x07 << 1)], #M1
		[0xC1,(0x1C << 1),0x70,(0x07 << 1)], #M2
		[0xC1,(0x1C << 1),0x70,(0x07 << 1)]] #M3


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchUnix()
        except ImportError:
	    print "error"
            #self.impl = _GetchWindows()
    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def lrotate(x):
	return (0xff & (((0x80 & x) >> 0x7) | (x << 1)))
def rrotate(x):
	return (0xff & (((0x01 & x) << 0x7) | (x >> 1)))
		

# A, nA drive
# index = M0-M3
def A(index):
	return VSET | (step[index][1] & 0x2) | (step[index][0] & 0x1) 
# B, nB drive
def B(index):
	return VSET | (step[index][3] & 0x2) | (step[index][2] & 0x1) 

def cwstep(index):
	step[index][0] = lrotate(step[index][0])
	step[index][1] = lrotate(step[index][1])
	step[index][2] = lrotate(step[index][2])
	step[index][3] = lrotate(step[index][3])
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))

def ccwstep(index):
	step[index][0] = rrotate(step[index][0])
	step[index][1] = rrotate(step[index][1])
	step[index][2] = rrotate(step[index][2])
	step[index][3] = rrotate(step[index][3])
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))

def printstep(index):		
	print "%x %x %x %x"%(0x1&step[index][0],(0x2&step[index][1])>>1,0x1&step[index][2],(0x2&step[index][3])>>1)

def motor_release():
	print "MOTOR RELEASE!"
        address = addressA
	bus.write_byte_data(address[0],0,0x18)
	bus.write_byte_data(address[1],0,0x18)
        address = addressB
	bus.write_byte_data(address[0],0,0x18)
	bus.write_byte_data(address[1],0,0x18)

def motor_lock(index):
	print "MOTOR LOCKED!"
        address = addressA
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))
        address = addressB
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))
	

def sigint_handler(signo, frame):
	print "received SIGINT."
	motor_release()
	exit(0)

signal.signal(signal.SIGINT,sigint_handler)

dt = 0.002
t=0
h=0
#step = FULL_STEP()
print "Motor running during push"
print " 8:up 2:down 5:lock 1:motor1 3:motor2 anynum:release char:exit"
while True:
	try:
    		getch = _Getch()
    		x = getch()
    		if (int(x) == 4):
                    address = addressA
                    for i in range(0,pulse):
			t = t + 1
    		    	cwstep(0)
                        time.sleep(dt)
    		        print t
    		elif (int(x) == 6):
                    address = addressA
                    for i in range(0,pulse):
			t = t - 1
    		    	ccwstep(0)
                        time.sleep(dt)
    		        print t
    		elif (int(x) == 5):
			motor_lock(0)
    		elif (int(x) == 2):
                    address = addressB
                    for i in range(0,pulse):
			h = h - 1
    		    	cwstep(0)
                        time.sleep(dt)
    		        print h
    		elif (int(x) == 8):
                    address = addressB
                    for i in range(0,pulse):
			h = h + 1
    		    	ccwstep(0)
                        time.sleep(dt)
    		        print h
    		else:
			motor_release()
    			t=0
        #	time.sleep(dt)
	except:
		motor_release()
		exit(0)

#	signal.pause()



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
motorstep step[4] = {
	{INIT_A, INIT_nA, INIT_B, INIT_nB},
	{INIT_A, INIT_nA, INIT_B, INIT_nB},
	{INIT_A, INIT_nA, INIT_B, INIT_nB},
	{INIT_A, INIT_nA, INIT_B, INIT_nB}
};

unsigned char oldA,oldB;
uint32_t stepcnt[4] = {0,0,0,0};
unsigned const char drv8830_addr_A[4] = {0xC0, 0xC4, 0xC8, 0xCC};
unsigned const char drv8830_addr_B[4] = {0xC2, 0xC6, 0xCA, 0xCE};

//void init(){
//}
//

void init_step(){
}
void updateReg(uint8_t num){
	drvreg[num].CONTROL_A = VSET | (step[num].A  & 0x1) | (step[num].nA & 0x2);
	drvreg[num].CONTROL_B = VSET | (step[num].B  & 0x1) | (step[num].nB & 0x2);
	  // I2C_MstSend( LPC_I2C, SE95_ADDR, (uint8_t *)I2CMasterTXBuffer, 2 );
	  // I2C_MstSend( LPC_I2C, SE95_ADDR, (uint8_t *)I2CMasterTXBuffer, 2 );
	++stepcnt[num];
}
void motor_release(){
	print "MOTOR RELEASE!"
        address = addressA
	bus.write_byte_data(address[0],0,0x18)
	bus.write_byte_data(address[1],0,0x18)
        address = addressB
	bus.write_byte_data(address[0],0,0x18)
	bus.write_byte_data(address[1],0,0x18)
}
void motor_lock(index){
	print "MOTOR LOCKED!"
        address = addressA
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))
        address = addressB
	bus.write_byte_data(address[0],0,A(index))
	bus.write_byte_data(address[1],0,B(index))
}
void fstep(uint8_t num){
	lrotate(step[num].A);
	lrotate(step[num].nA);
	lrotate(step[num].B);
	lrotate(step[num].nB);
	updateReg(num);
}
void rstep(uint8_t num){
	rrotate(step[num].A);
	rrotate(step[num].nA);
	rrotate(step[num].B);
	rrotate(step[num].nB);
	updateReg(num);
}
void fdeg(uint32_t deg, uint8_t num){
	uint32_t loop = deg/DSTEP;
	uint32_t i=0;
	for(i=0;i<loop;i++){
		fstep(num);
//		wait();
	}

}


int main()
{
	int fd,ret;
	int ID;

	= 0x60;
	ID = 0x60;

	/* WHO AM I */
	fd = wiringPiI2CSetup(ID);
	printf("setup return : %d\n",fd);
	
	/* start senser */
//	if((wiringPiI2CWriteReg8(fd,0x20,0x0F))<0){
//		printf("write error register 0x20");
//	}
//	printf("write register:0x20 = 0x0F\n");
	
	wiringPiI2CWriteReg8(fd,0x0,(0x06<<2)|0x3);
	/* read OUT_X_L */
	ret = wiringPiI2CReadReg8(fd,0x0);
	printf("OUT_X_L : %d\n",ret);
	
	wiringPiI2CWriteReg8(fd,0x0,0x0);
	
	return;
}
