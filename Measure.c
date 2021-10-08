//Student ID: B073040006


//port0, pin8 has changed into port0, pin6
//port0, pin9 has changed into port0, pin7
#include <stdio.h>                       /* standard I/O .h-file              */
#include <ctype.h> /* character functions               */
#ifndef MCB2130
  #include <LPC21xx.H>                   /* LPC21xx definitions               */
#else
  #include <LPC213x.H>                   /* LPC213x definitions               */
  #define  ADCR    AD0CR
  #define  ADDR    AD0DR
#endif

#include "measure.h"                     /* global project definition file    */


const char menu[] =
   "\n"
   "+***************** REMOTE MEASUREMENT RECORDER *****************+\n"
   "| This program is a simple Measurement Recorder. It is based on |\n"
   "| the LPC2129 and records the state of Port 0 and the voltage   |\n"
   "| on the four analog inputs AIN0 trough AIN3.                   |\n"
   "+ command -+ syntax -----+ function ----------------------------+\n"
   "| Read     | R [n]       | read <n> recorded measurements       |\n"
   "| Display  | D           | display current measurement values   |\n"
   "| Time     | T hh:mm:ss  | set time                             |\n"
   "| Interval | I mm:ss.ttt | set interval time                    |\n"
   "| Clear    | C           | clear measurement records            |\n"
   "| Quit     | Q           | quit measurement recording           |\n"
   "| Start    | S           | start measurement recording          |\n"
   "+----------+-------------+--------------------------------------+\n";

struct interval setinterval;                /* interval setting values        */
struct interval interval;                   /* interval counter               */

volatile int measurement_interval = 0;      /* measurement interval over      */
volatile int mdisplay = 0;                  /* measurement display requested  */
volatile int startflag = 0;                 /* start measurement recording    */

struct mrec current;                        /* current measurements           */

#define SCNT  6                             /* number of records              */

struct mrec save_record[SCNT];              /* buffer for measurements        */
int sindex;                                 /* save index                     */
int savefirst;                              /* save first index               */

char ERROR [] = "\n*** ERROR: %s\n";        /* ERROR message string in code   */

#define WRONGINDEX 0xffff                   /* error signal for wrong index   */

int hourglassl=0,hourglassr=0;
unsigned long a=0x00000000,b=0x00800080,temp;
int checkl=1,checkr=1;
char left[8][10],tl[8][10],right[8][10],tr[8][10];
char levell[4],levelr[4];
/******************************************************************************/
/*               Save current measurements in save_record                     */
/******************************************************************************/


/* Default Interrupt Function: may be called when timer ISR is disabled */
void DefISR (void) __irq  {
  ;
}

void elevator(int n)
{
 if(n==0)  //left elevator
 {
	if((b&0x80000000)==0x80000000) //3Fclosed
	{
		if((a&0x00000040)==0x00000040 | (a&0x00000800)==0x00000800 | (a&0x00080000)==0x00080000 | ((a&0x00000800)==0x00000800 & (a&0x00080000)==0x00080000))
		{
			b = 0x40000000;
			IOSET1 = 3<<28;  //opening door
		}
		else if((a&0x00010000)==0x00010000 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000 | (a&0x00000080)==0x00000080 | (a&0x00000400)==0x00000400)
		{
			b = 0x00080000;  //going downstairs
			IOSET1 = 1<<28;
		}
	}
	else if((b&0x40000000)==0x40000000)  //3F opening door
	{
		
		if((IOPIN0&0x00100000)==0x00100000)
		{
			b = 0x10000000;   //3F door open
			IOCLR1 = 3<<28;
			if((a&0x00000040)==0x00000040)
				IOCLR1 = 1<<16;
			if((a&0x00000800)==0x00000800)
				IOCLR1 = 1<<19;
			if((a&0x00080000)==0x00080000)
				IOCLR1 = 1<<27;
			a = a & 0xFFF7F7BF;  //clean L<|> L3 3v 
		}
	}
	else if((b&0x10000000)==0x10000000)   //if 3F door open
	{
		hourglassl++;
		if(hourglassl==5000)
		{
			b = 0x20000000;   //3F door closing
			hourglassl=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x20000000)==0x20000000)  //3F closing door
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			b = 0x80000000;    //3F door closed
			IOCLR1 = 3<<28;
		}
	}
	
	else if((b&0x00080000)==0x00080000)   //if 2.5F v
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<28;
			b = 0x08000000;      //2F door closed
		}
	}
	else if((b&0x08000000)==0x08000000)  //2F door closed
	{
		if((a&0x00000040)==0x00000040 | (a&0x00000400)==0x00000400 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000)
		{
			b = 0x04000000;  //2F door opening
			IOSET1 = 3<<28;
		}
		else if((a&0x00000080)==0x00000080 | (a&0x00010000)==0x00010000)
 		{
			b = 0x00040000;  //going downstairs
			IOSET1 = 1<<28;
		}
    else if((a&0x00000800)==0x00000800 | (a&0x00080000)==0x00080000)
		{
			b = 0x00010000;  //going upstairs
			IOSET1 = 1<<29;
		}			
	}
	else if((b&0x04000000)==0x04000000)  
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x01000000;   //2F door open
			if((a&0x00000040)==0x00000040)
				IOCLR1 = 1<<16;
			if((a&0x00000400)==0x00000400)
				IOCLR1 = 1<<18;
			if((a&0x00020000)==0x00020000)
				IOCLR1 = 1<<25;
			if((a&0x00040000)==0x00040000)
				IOCLR1 = 1<<26;
			a = a & 0xFFF9FBBF;
		}
	}
	else if((b&0x00040000)==0x00040000)  //if 1.5F v
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<28;
			b = 0x00800000;   //1F door closed
		}
	}
	else if((b&0x00010000)==0x00010000)  //if 2.5F ^
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 1<<29;
			b = 0x80000000;   //3F door closed
		}
	}
	else if((b&0x01000000)==0x01000000)  //2F open
	{
		hourglassl++;
		if(hourglassl==5000)
		{
			b = 0x02000000;   //2F door closing
			hourglassl=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x02000000)==0x02000000)  //if 2F closing door
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x08000000;  //2F door closed
		}
	}
	else if((b&0x00800000)==0x00800000)  //1F door closed
	{
		if((a&0x00000040)==0x00000040 | (a&0x00000080)==0x00000080 | (a&0x00010000)==0x00010000 | ((a&0x00000080)==0x00000080 & (a&0x00010000)==0x00010000))
		{
			b = 0x00400000;  //1F door opening
			IOSET1 = 3<<28;
		}
		else if((a&0x00080000)==0x00080000 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000 | (a&0x00000400)==0x00000400 | (a&0x00000800)==0x00000800)
		{
			b = 0x00020000;  //1.5F ^
			IOSET1 = 1<<29;
		}
	}
	else if((b&0x00400000)==0x00400000)
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00100000;  //1F door open
			if((a&0x00000040)==0x00000040)
				IOCLR1 = 1<<16;
			if((a&0x00000080)==0x00000080)
				IOCLR1 = 1<<17;
			if((a&0x00010000)==0x00010000)
				IOCLR1 = 1<<24;
			a = a & 0xFFFEFF3F;
		}
	}
	else if((b&0x00100000)==0x00100000)
	{
		hourglassl++;
		if(hourglassl==5000)
		{
			b = 0x00200000;   //1F door closing
			hourglassl=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x00200000)==0x00200000)
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00800000;  //1F door closed
		}
	}
	else if((b&0x00020000)==0x00020000) //if 1.5F ^
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<29;
			b = 0x08000000;
		}
	}
 }
 else if(n==1)  //right elevator
 {
	if((b&0x00008000)==0x00008000) //3Fclosed
	{
		if((a&0x00001000)==0x00001000 | (a&0x00008000)==0x00008000 | (a&0x00080000)==0x00080000 | ((a&0x00008000)==0x00008000 & (a&0x00080000)==0x00080000))
		{
			b = 0x00004000;
			IOSET1 = 3<<28;  //opening door
		}
		else if((a&0x00010000)==0x00010000 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000 | (a&0x00002000)==0x00002000 | (a&0x00004000)==0x00004000)
		{
			b = 0x00000008;  //going downstairs
			IOSET1 = 1<<28;
		}
	}
	else if((b&0x00004000)==0x00004000)  //3F opening door
	{
		
		if((IOPIN0&0x00100000)==0x00100000)
		{
			b = 0x00001000;   //3F door open
			IOCLR1 = 3<<28;
			if((a&0x00001000)==0x00001000)
				IOCLR1 = 1<<16;
			if((a&0x00008000)==0x00008000)
				IOCLR1 = 1<<19;
			if((a&0x00080000)==0x00080000)
				IOCLR1 = 1<<27;
			a = a & 0xFFF76FFF;  //clean R<|> R3 3v 
		}
	}
	else if((b&0x00001000)==0x00001000)   //if 3F door open
	{
		hourglassr++;
		if(hourglassr==5000)
		{
			b = 0x00002000;   //3F door closing
			hourglassr=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x00002000)==0x00002000)  //3F closing door
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			b = 0x00008000;    //3F door closed
			IOCLR1 = 3<<28;
		}
	}
	
	else if((b&0x00000008)==0x00000008)   //if 2.5F v
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<28;
			b = 0x00000800;      //2F door closed
		}
	}
	else if((b&0x00000800)==0x00000800)  //2F door closed
	{
		if((a&0x00001000)==0x00001000 | (a&0x00002000)==0x00002000 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000)
		{
			b = 0x00000400;  //2F door opening
			IOSET1 = 3<<28;
		}
		else if((a&0x00002000)==0x00002000 | (a&0x00010000)==0x00010000)
 		{
			b = 0x00000004;  //going downstairs
			IOSET1 = 1<<28;
		}
    else if((a&0x00008000)==0x00008000 | (a&0x00080000)==0x00080000)
		{
			b = 0x00000001;  //going upstairs
			IOSET1 = 1<<29;
		}			
	}
	else if((b&0x00004000)==0x00004000)  
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00000100;   //2F door open
			if((a&0x00001000)==0x00001000)
				IOCLR1 = 1<<16;
			if((a&0x00004000)==0x00004000)
				IOCLR1 = 1<<18;
			if((a&0x00020000)==0x00020000)
				IOCLR1 = 1<<25;
			if((a&0x00040000)==0x00040000)
				IOCLR1 = 1<<26;
			a = a & 0xFFF9AFFF;
		}
	}
	else if((b&0x00000004)==0x00000004)  //if 1.5F v
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<28;
			b = 0x00000080;   //1F door closed
		}
	}
	else if((b&0x00000001)==0x00000001)  //if 2.5F ^
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 1<<29;
			b = 0x00008000;   //3F door closed
		}
	}
	else if((b&0x00000100)==0x00000100)  //2F open
	{
		hourglassr++;
		if(hourglassr==5000)
		{
			b = 0x00000200;   //2F door closing
			hourglassr=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x00000200)==0x00000200)  //if 2F closing door
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00000800;  //2F door closed
		}
	}
	else if((b&0x00000080)==0x00000080)  //1F door closed
	{
		if((a&0x00001000)==0x00001000 | (a&0x00002000)==0x00002000 | (a&0x00010000)==0x00010000 | ((a&0x00002000)==0x00002000 & (a&0x00010000)==0x00010000))
		{
			b = 0x00000040;  //1F door opening
			IOSET1 = 3<<28;
		}
		else if((a&0x00080000)==0x00080000 | (a&0x00020000)==0x00020000 | (a&0x00040000)==0x00040000 | (a&0x00004000)==0x00004000 | (a&0x00008000)==0x00008000)
		{
			b = 0x00000002;  //1.5F ^
			IOSET1 = 1<<29;
		}
	}
	else if((b&0x00000040)==0x00000040)
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00000010;  //1F door open
			if((a&0x00001000)==0x00001000)
				IOCLR1 = 1<<16;
			if((a&0x00002000)==0x00002000)
				IOCLR1 = 1<<17;
			if((a&0x00010000)==0x00010000)
				IOCLR1 = 1<<24;
			a = a & 0xFFFECFFF;
		}
	}
	else if((b&0x00000010)==0x00000010)
	{
		hourglassr++;
		if(hourglassr==5000)
		{
			b = 0x00000020;   //1F door closing
			hourglassr=0;
			IOSET1 = 3<<28;
		}
	}
	else if((b&0x00000020)==0x00000020)
	{
		if((IOPIN0&0x00100000)==0x00100000)
		{
			IOCLR1 = 3<<28;
			b = 0x00000080;  //1F door closed
		}
	}
	else if((b&0x00000002)==0x00000002) //if 1.5F ^
	{
		if((IOPIN0&0x00200000)==0x00000000)
		{
			IOCLR1 = 1<<29;
			b = 0x00000800;
		}
	}
 }
	
}

__irq void tc0 (void) {
	  
	  a|=IOPIN0;
	  IOSET1=(a<<8);
		elevator(0);  //left elevator
	  elevator(1);  //right elevator
	
    T0IR = 1;                                    /* Clear interrupt flag        */
    VICVectAddr = 0;                             /* Acknowledge Interrupt       */
}

void display(int n)
{
	int u,v;
	if(n==0)
	{
		printf("elevator #0\n");
		for(u = 0; u < 8; u++)
			{
				for(v = 0; v < 10; v++)
					printf("%c",left[u][v]);
				printf("\n");
			}
			printf("---------------\n");
			for(u=0;u<8;u++)
			{
				for(v=0;v<10;v++)
				  tl[u][v]=left[u][v];
			}
			checkl=0;
	}
	else if(n==1)
	{
		printf("elevator #1\n");
		for(u = 0; u < 8; u++)
			{
				for(v = 0; v < 10; v++)
					printf("%c",right[u][v]);
				printf("\n");
			}
			printf("---------------\n");
			for(u=0;u<8;u++)
			{
				for(v=0;v<10;v++)
				  tr[u][v]=right[u][v];
			}
			checkr=0;
	}
}

int main (void)  {                             /* main entry for program      */
  
  int i,j;
  PINSEL1 = 0x15400000;                        /* Select AIN0..AIN3           */
  IODIR1  = 0xFFFF0000;                        /* P1.16..23 defined as Outputs*/
  ADCR    = 0x002E0401;                        /* Setup A/D: 10-bit @ 3MHz    */

  init_serial ();                              /* initialite serial interface */

  /* setup the timer counter 0 interrupt */
  T0MR0 = 14999;                               /* 1mSec = 15.000-1 counts     */
  T0MCR = 3;                                   /* Interrupt and Reset on MR0  */
  T0TCR = 1;                                   /* Timer0 Enable               */
  VICVectAddr0 = (unsigned long)tc0;           /* set interrupt vector in 0   */
  VICVectCntl0 = 0x20 | 4;                     /* use it for Timer 0 Interrupt*/
  VICIntEnable = 0x00000010;                   /* Enable Timer0 Interrupt     */
	
	
	
	for(i = 0; i < 8; i++)
		{
			for(j = 0; j < 10; j++)
			{
				tl[i][j] = ' ';
				tr[i][j] = ' ';
			}
		}

  while(1)
	{
		for(i = 0; i < 8; i++)
		{
			for(j = 0; j < 10; j++)
			{
				left[i][j] = ' ';
				right[i][j] = ' ';
			}
		}
	
		for(i=0;i<4;i++)
		{
			levell[i] = ' ';
			levelr[i] = ' ';
		}
				
		left[0][7] = '[';left[0][9] = ']';
		left[3][7] = '[';left[3][9] = ']';
		left[6][7] = '[';left[6][9] = ']';
		
		right[0][7] = '[';right[0][9] = ']';
		right[3][7] = '[';right[3][9] = ']';
		right[6][7] = '[';right[6][9] = ']';
		
		
		if((a&0x00010000)==0x00010000)
			left[6][8] = '^';
		if((a&0x00020000)==0x00020000 && (a&0x00040000)==0x00040000)
			left[3][8] = 'X';
		else if((a&0x00020000)==0x00020000)
			left[3][8] = '^';
		else if((a&0x00040000)==0x00040000)
			left[3][8] = 'v';
		if((a&0x00080000)==0x00080000)
			left[0][8] = 'v';
		if((a&0x00000080)==0x00000080)
			levell[1] = '1';
		if((a&0x00000400)==0x00000400)
			levell[2] = '2';
		if((a&0x00000800)==0x00000800)
			levell[3] = '3';
		
		if((b&0x00010000)==0x00010000)//2.5^
		{
			left[1][0] = '[';
			left[1][1] = levell[1];
			left[1][2] = levell[2];
			left[1][3] = levell[3];
			left[1][4]=']';
			left[2][0] = '^';
			left[2][1] = '^';
			left[2][2] = '^';
			left[2][3] = '^';
			left[2][4] = '^'; 
		}
		else if((b&0x00020000)==0x00020000)//1.5^
		{
			left[4][0] = '[';
			left[4][1] = levell[1];
			left[4][2] = levell[2];
			left[4][3] = levell[3];
			left[4][4]=']';
			left[5][0] = '^';
			left[5][1] = '^';
			left[5][2] = '^';
			left[5][3] = '^';
			left[5][4] = '^';
		}
		else if((b&0x00040000)==0x00040000)//1.5v
		{
			left[4][0] = '[';
			left[4][1] = levell[1];
			left[4][2] = levell[2];
			left[4][3] = levell[3];
			left[4][4]=']';
			left[5][0] = 'v';
			left[5][1] = 'v';
			left[5][2] = 'v';
			left[5][3] = 'v';
			left[5][4] = 'v';
		}
		else if((b&0x00080000)==0x00080000)//2.5v
		{
			left[1][0] = '[';
			left[1][1] = levell[1];
			left[1][2] = levell[2];
			left[1][3] = levell[3];
			left[1][4]=']';
			left[2][0] = 'v';
			left[2][1] = 'v';
			left[2][2] = 'v';
			left[2][3] = 'v';
			left[2][4] = 'v'; 
		}
		else if((b&0x00100000)==0x00100000)//1Fopen
		{
			left[6][0] = '[';
			left[6][1] = levell[1];
			left[6][2] = levell[2];
			left[6][3] = levell[3];
			left[6][4] = ']';
			left[7][0] = '|';
			left[7][1] = '=';
			left[7][2] = '=';
			left[7][3] = '=';
			left[7][4] = '|';
		}
		else if((b&0x01000000)==0x01000000)  //2Fopen
		{
			left[3][0] = '[';
			left[3][1] = levell[1];
			left[3][2] = levell[2];
			left[3][3] = levell[3];
			left[3][4] = ']';
			left[4][0] = '|';
			left[4][1] = '=';
			left[4][2] = '=';
			left[4][3] = '=';
			left[4][4] = '|';
		}
		else if((b&0x10000000)==0x10000000)  //3Fopen
		{
			left[0][0] = '[';
			left[0][1] = levell[1];
			left[0][2] = levell[2];
			left[0][3] = levell[3];
			left[0][4] = ']';
			left[1][0] = '|';
			left[1][1] = '=';
			left[1][2] = '=';
			left[1][3] = '=';
			left[1][4] = '|';
		}
		else if((b&0x00200000)==0x00200000)//1F><
		{
			left[6][0] = '[';
			left[6][1] = levell[1];
			left[6][2] = levell[2];
			left[6][3] = levell[3];
			left[6][4] = ']';
			left[7][1] = '>';
			left[7][2] = '|';
			left[7][3] = '<';
		}
		else if((b&0x02000000)==0x02000000)  //2F><
		{
			left[3][0] = '[';
			left[3][1] = levell[1];
			left[3][2] = levell[2];
			left[3][3] = levell[3];
			left[3][4] = ']';
			left[4][1] = '>';
			left[4][2] = '|';
			left[4][3] = '<';
		}
		else if((b&0x20000000)==0x20000000)  //3F><
		{
			left[0][0] = '[';
			left[0][1] = levell[1];
			left[0][2] = levell[2];
			left[0][3] = levell[3];
			left[0][4] = ']';
			left[1][1] = '>';
			left[1][2] = '|';
			left[1][3] = '<';
		}
		else if((b&0x00400000)==0x00400000)//1F<>
		{
			left[6][0] = '[';
			left[6][1] = levell[1];
			left[6][2] = levell[2];
			left[6][3] = levell[3];
			left[6][4] = ']';
			left[7][0] = '|';
			left[7][1] = '<';
			left[7][2] = '=';
			left[7][3] = '>';
			left[7][4] = '|';
		}
		else if((b&0x02000000)==0x02000000)  //2F<>
		{
			left[3][0] = '[';
			left[3][1] = levell[1];
			left[3][2] = levell[2];
			left[3][3] = levell[3];
			left[3][4] = ']';
			left[4][0] = '|';
			left[4][1] = '<';
			left[4][2] = '=';
			left[4][3] = '>';
			left[4][4] = '|';
		}
		else if((b&0x20000000)==0x20000000)  //3F<>
		{
			left[0][0] = '[';
			left[0][1] = levell[1];
			left[0][2] = levell[2];
			left[0][3] = levell[3];
			left[0][4] = ']';
			left[1][0] = '|';
			left[1][1] = '<';
			left[1][2] = '=';
			left[1][3] = '>';
			left[1][4] = '|';
		}
		else if((b&0x00800000)==0x00800000)//1Fclosed
		{
			left[6][0] = '[';
			left[6][1] = levell[1];
			left[6][2] = levell[2];
			left[6][3] = levell[3];
			left[6][4] = ']';
			left[7][2] = '|';
		}
		else if((b&0x02000000)==0x02000000)  //2Fclosed
		{
			left[3][0] = '[';
			left[3][1] = levell[1];
			left[3][2] = levell[2];
			left[3][3] = levell[3];
			left[3][4] = ']';
			left[4][2] = '|';
		}
		else if((b&0x20000000)==0x20000000) //3Fclosed
		{
			left[0][0] = '[';
			left[0][1] = levell[1];
			left[0][2] = levell[2];
			left[0][3] = levell[3];
			left[0][4] = ']';
			left[1][2] = '|';
		}
		
		for(i=0;i<7;i++)
		{
			for(j=0;j<10;j++)
			{
				if(tl[i][j]!=left[i][j]&&(left[0][0]!=' '||left[1][0]!=' '||left[3][0]!=' '||left[4][0]!=' '||left[6][0]!=' '||left[7][0]))
					checkl=1;
			}
		}
		
		
		
		if((a&0x00010000)==0x00010000)
			right[6][8] = '^';
		if((a&0x00020000)==0x00020000 && (a&0x00040000)==0x00040000)
			right[3][8] = 'X';
		else if((a&0x00020000)==0x00020000)
			right[3][8] = '^';
		else if((a&0x00040000)==0x00040000)
			right[3][8] = 'v';
		if((a&0x00080000)==0x00080000)
			right[0][8] = 'v';
		if((a&0x00002000)==0x00002000)
			levelr[1] = '1';
		if((a&0x00004000)==0x00004000)
			levelr[2] = '2';
		if((a&0x00008000)==0x00008000)
			levelr[3] = '3';
		
		if((b&0x00000001)==0x00000001)//2.5^
		{
			right[1][0] = '[';
			right[1][1] = levelr[1];
			right[1][2] = levelr[2];
			right[1][3] = levelr[3];
			right[1][4]=']';
			right[2][0] = '^';
			right[2][1] = '^';
			right[2][2] = '^';
			right[2][3] = '^';
			right[2][4] = '^'; 
		}
		else if((b&0x00000002)==0x00000002)//1.5^
		{
			right[4][0] = '[';
			right[4][1] = levelr[1];
			right[4][2] = levelr[2];
			right[4][3] = levelr[3];
			right[4][4]=']';
			right[5][0] = '^';
			right[5][1] = '^';
			right[5][2] = '^';
			right[5][3] = '^';
			right[5][4] = '^';
		}
		else if((b&0x00000004)==0x00000004)//1.5v
		{
			right[4][0] = '[';
			right[4][1] = levelr[1];
			right[4][2] = levelr[2];
			right[4][3] = levelr[3];
			right[4][4]=']';
			right[5][0] = 'v';
			right[5][1] = 'v';
			right[5][2] = 'v';
			right[5][3] = 'v';
			right[5][4] = 'v';
		}
		else if((b&0x00000008)==0x00000008)//2.5v
		{
			right[1][0] = '[';
			right[1][1] = levelr[1];
			right[1][2] = levelr[2];
			right[1][3] = levelr[3];
			right[1][4]=']';
			right[2][0] = 'v';
			right[2][1] = 'v';
			right[2][2] = 'v';
			right[2][3] = 'v';
			right[2][4] = 'v'; 
		}
		else if((b&0x00000010)==0x00000010)//1Fopen
		{
			right[6][0] = '[';
			right[6][1] = levelr[1];
			right[6][2] = levelr[2];
			right[6][3] = levelr[3];
			right[6][4] = ']';
			right[7][0] = '|';
			right[7][1] = '=';
			right[7][2] = '=';
			right[7][3] = '=';
			right[7][4] = '|';
		}
		else if((b&0x00000100)==0x00000100)  //2Fopen
		{
			right[3][0] = '[';
			right[3][1] = levelr[1];
			right[3][2] = levelr[2];
			right[3][3] = levelr[3];
			right[3][4] = ']';
			right[4][0] = '|';
			right[4][1] = '=';
			right[4][2] = '=';
			right[4][3] = '=';
			right[4][4] = '|';
		}
		else if((b&0x00001000)==0x00001000)  //3Fopen
		{
			right[0][0] = '[';
			right[0][1] = levelr[1];
			right[0][2] = levelr[2];
			right[0][3] = levelr[3];
			right[0][4] = ']';
			right[1][0] = '|';
			right[1][1] = '=';
			right[1][2] = '=';
			right[1][3] = '=';
			right[1][4] = '|';
		}
		else if((b&0x00000020)==0x00000020)//1F><
		{
			right[6][0] = '[';
			right[6][1] = levelr[1];
			right[6][2] = levelr[2];
			right[6][3] = levelr[3];
			right[6][4] = ']';
			right[7][1] = '>';
			right[7][2] = '|';
			right[7][3] = '<';
		}
		else if((b&0x00000200)==0x00000200)  //2F><
		{
			right[3][0] = '[';
			right[3][1] = levelr[1];
			right[3][2] = levelr[2];
			right[3][3] = levelr[3];
			right[3][4] = ']';
			right[4][1] = '>';
			right[4][2] = '|';
			right[4][3] = '<';
		}
		else if((b&0x00002000)==0x00002000)  //3F><
		{
			right[0][0] = '[';
			right[0][1] = levelr[1];
			right[0][2] = levelr[2];
			right[0][3] = levelr[3];
			right[0][4] = ']';
			right[1][1] = '>';
			right[1][2] = '|';
			right[1][3] = '<';
		}
		else if((b&0x00000040)==0x00000040)//1F<>
		{
			right[6][0] = '[';
			right[6][1] = levelr[1];
			right[6][2] = levelr[2];
			right[6][3] = levelr[3];
			right[6][4] = ']';
			right[7][0] = '|';
			right[7][1] = '<';
			right[7][2] = '=';
			right[7][3] = '>';
			right[7][4] = '|';
		}
		else if((b&0x00000200)==0x00000200)  //2F<>
		{
			right[3][0] = '[';
			right[3][1] = levelr[1];
			right[3][2] = levelr[2];
			right[3][3] = levelr[3];
			right[3][4] = ']';
			right[4][0] = '|';
			right[4][1] = '<';
			right[4][2] = '=';
			right[4][3] = '>';
			right[4][4] = '|';
		}
		else if((b&0x00002000)==0x00002000)  //3F<>
		{
			right[0][0] = '[';
			right[0][1] = levelr[1];
			right[0][2] = levelr[2];
			right[0][3] = levelr[3];
			right[0][4] = ']';
			right[1][0] = '|';
			right[1][1] = '<';
			right[1][2] = '=';
			right[1][3] = '>';
			right[1][4] = '|';
		}
		else if((b&0x00000080)==0x00000080)//1Fclosed
		{
			right[6][0] = '[';
			right[6][1] = levelr[1];
			right[6][2] = levelr[2];
			right[6][3] = levelr[3];
			right[6][4] = ']';
			right[7][2] = '|';
		}
		else if((b&0x00000200)==0x00000200)  //2Fclosed
		{
			right[3][0] = '[';
			right[3][1] = levelr[1];
			right[3][2] = levelr[2];
			right[3][3] = levelr[3];
			right[3][4] = ']';
			right[4][2] = '|';
		}
		else if((b&0x00002000)==0x00002000) //3Fclosed
		{
			right[0][0] = '[';
			right[0][1] = levelr[1];
			right[0][2] = levelr[2];
			right[0][3] = levelr[3];
			right[0][4] = ']';
			right[1][2] = '|';
		}
		
		for(i=0;i<7;i++)
		{
			for(j=0;j<10;j++)
			{
				if(tr[i][j]!=right[i][j]&&(right[0][0]!=' '||right[1][0]!=' '||right[3][0]!=' '||right[4][0]!=' '||right[6][0]!=' '||right[7][0]))
					checkr=1;
			}
		}
		
		if(checkl)
			display(0);
		if(checkr)
			display(1);
	}
}


