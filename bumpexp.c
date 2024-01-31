//sudo nice -n -20 ./bumpexp .80 

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <fbiad.h>
#include <fbida.h>
#include <unistd.h> // for usleep(
#include <termios.h> // for kbhit()
#include <fcntl.h> // for kbhit()
#include <inttypes.h>
#include <pthread.h>

//#include "rt_waitDA.h"
//#include "rt_waitAD.h"
#include "rt_waitADDA.h"


ADBOARDSPEC BoardSpec;
ADSMPLREQ AdSmplConfig;
ADSMPLCHREQ AdSmplChReq[12];

DABOARDSPEC Spec;
DASMPLREQ SmplConfig;
DASMPLCHREQ DaSmplChReq[2];

unsigned short Data[12];
double Data1[12];
double Data2[12];
unsigned short Output[2];
double stimuli[4], signal[2];
#define TestCount 1000000
#define filn 30
#define count1 500

double AM_F =3800.0; //Hz of AM (2kHz = 35000)

double rt_cycleAD = 20.0; //ms 80Hz 12.5 Hz
double rt_cycleDA = 0.5; //ms 2000Hz

double rx,ry;
double dr;
double dx,dy,dxn,dyn,drn; 

double x[TestCount];
double y[TestCount];
double xv[TestCount];
double yv[TestCount];
double xf[TestCount];
double yf[TestCount];

double nf=0;
double nfp=0;
double nfd,nfds,nfr,nfrp;
double mu = 0; // coefficient of friction between finger pad and panel
double mu_m = 0; // coefficient of friction between finger pad and each natural material
double mup = 0;
//////////////////////
double mu_p = 0.10;
double k_p = 3;
//////////////////////
double k = 0;
int j;
int ret;
int nRet;
double a = 1.0*1.0;
double surf = 1.0;
double ga = 0.4;
double freqv1 = 1.0;
double phyv1 = 0;
double phyvp1 = 0;
double phyxp1 = 0;
double ampv1 = 0;
double ampe1 = 0;
double Plen = 20;
double alpha = 1.0;
int BUTTON = 0;
int SIN = 0;
double output;
double power;
double load=0;
double sf=0; //shear force
double ave_nf = 0, ave_sf = 0, ave_k=0; 
double ave_x = 0, ave_y = 0;
double ave_xp=0, ave_yp=0;
double dx,dxa,dy,dya;
double dv = 0.1;
double newvalue_x, newvalue_y,newvalue_xp, newvalue_yp, newvalue_f;
float x_history[filn], y_history[filn];
float nf_history[count1],sf_history[count1];

char c = '0';
int j=0,nof;
double f1,f2,f3;//TestCount
int start=TestCount;

int g_quit = 0;
char c;

//reproduction kbhit in Linux
int kbhit(void) 
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}


//pthreadによるDA制御
void *Out(void *p){

	int i = 0;
	int SURFACE = 1;
	

	// For saving data
	FILE *fp1;
	fp1 = fopen( "DA.csv", "w" );
	
	unsigned long long rt_ticks = GetTick();	
	nRet = DaOpen(1);
	if(nRet){
		printf("Open error: ret=%Xh\n", nRet);
		exit(EXIT_FAILURE);
	}
	nRet = DaGetDeviceInfo(1, &Spec);
	if(nRet){
		printf("DaGetDeviceInfo error: ret=%Xh\n", nRet);
		DaClose(1);
		exit(EXIT_FAILURE);
	}	
	nRet = DaGetSamplingConfig(1, &SmplConfig);

	if (nRet) {
		printf("DaGetSamplingConfig error: ret=%Xh\n", nRet);
		DaClose(1);
		exit(EXIT_FAILURE);
	}
	//DA Channels
	DaSmplChReq[0].ulChNo = 1;
	DaSmplChReq[0].ulRange = DA_10V;
	DaSmplChReq[1].ulChNo = 2;
	DaSmplChReq[1].ulRange = DA_10V;
	
	

	while( i++ < TestCount ){

		ampv1 = 2.0;
		ampe1 = 1.2;

		if(nf<0){
		nf=0;
		}

		if (dv < 0.05){
			dv = 0;
		}
		else if (dv > 300){
			dv =300;
		}

		Output[0] = ampv1 * a * sin(2*M_PI*50.0*(float)i*(rt_cycleDA/1000.0))/ 10.0 * 0x8000 + 0x8000;

		double sgn = 0;
		double H_LAM = Plen;
		if( sin( (newvalue_x + H_LAM) * 2 * M_PI / (2 * H_LAM) ) > 0 ) {
			sgn = 1;
		}
		else if( sin( (newvalue_x + H_LAM) * 2 * M_PI / (2 * H_LAM) ) < 0 ) {
			sgn = -1;
		}
		else if( sin( (newvalue_x + H_LAM) * 2 * M_PI / (2 * H_LAM) ) == 0 ) {
			sgn = 0;
		}

		phyv1 =  (dv / freqv1) + phyvp1;  // (v*t + x(t-1)) / lambda, dv is distance moved
		double LAM = 2 * H_LAM;//波長
		double X = (newvalue_x + H_LAM) * 2 * M_PI / LAM;
		double MAX_EXP=0;
		

		double sigma = 1, extremum;
		double alpha,surf=1.0;
		double EX = newvalue_x / 10;
		
		if( SIN != 1) {
			switch(c){
				//expbump&dent
				case '2':
					sigma = 1;
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = sqrt(2) * exp(-1/2);
					//double X = 3 * newvalue_x / (2 * H_LAM);
					Output[1] = ga * a * 4.0 * sqrt( -2 * EX / MAX_EXP * exp( -pow( EX,2 ) / sigma) / sigma + 1.0 ) / 10.0 * 0x8000 + 0x8000;
					break;
				case '3':
					sigma = 0.5;// ここを1/2とするとエラーintと判断されてしまうため
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = 2 * exp(-1/2);
					Output[1] = ga * a * 4.0 * sqrt( -2 * EX / MAX_EXP * exp( -pow( EX,2 ) / sigma) / sigma + 1.0) / 10.0 * 0x8000 + 0x8000;
					break;
				case '1':
					sigma = 2;
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = sqrt(1/exp(1));
					Output[1] = ga * a * 4.0 * sqrt( -2 * EX / MAX_EXP * exp( -pow( EX,2 ) / sigma) / sigma + 1.0 ) / 10.0 * 0x8000 + 0x8000;			
					break;
				case 'b':
					sigma = 1;
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = sqrt(2) * exp(-1/2);
					//double X = 3 * newvalue_x / (2 * H_LAM);
					Output[1] = ga * a * 4.0 * sqrt( -2 * (-EX) / MAX_EXP * exp( -pow( -EX,2 ) / sigma) / sigma + 1.0 ) / 10.0 * 0x8000 + 0x8000;
					break;
				case 'c':
					sigma = 0.5;// ここを1/2とするとエラーintと判断されてしまうため
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = 2 * exp(-1/2);
					Output[1] = ga * a * 4.0 * sqrt( -2 * (-EX) / MAX_EXP * exp( -pow( -EX,2 ) / sigma) / sigma + 1.0) / 10.0 * 0x8000 + 0x8000;
					break;
				case 'a':
					sigma = 2;
					extremum = -sqrt(sigma*0.5);
					MAX_EXP = -2 * extremum * exp( -pow( extremum, 2 ) / sigma ) / sigma;
					//MAX_EXP = sqrt(1/exp(1));
					Output[1] = ga * a * 4.0 * sqrt( -2 * (-EX) / MAX_EXP * exp( -pow( -EX,2 ) / sigma) / sigma + 1.0 ) / 10.0 * 0x8000 + 0x8000;			
					break;
				default:
					Output[1] = 0x8000;
			}
		}
		else if(SIN == 1) {
			if( newvalue_f > 20.0 && newvalue_x > -H_LAM && newvalue_x < H_LAM ){
				switch(c){
					case 'g':
						alpha = 1.0;
						Output[1] = ga * a * 4.0 * sqrt( surf * sgn * pow( sgn * sin( X ) , alpha ) + 1.0 ) / 10.0 * 0x8000 + 0x8000;
						break;
					case 'h':
						alpha = 2.0;
						Output[1] = ga * a * 4.0 * sqrt( surf * sgn * pow( sgn * sin( X ) , alpha ) + 1.0 ) / 10.0 * 0x8000 + 0x8000;
						break;
					case 'j':
						alpha = 3.0;
						//sgn = 1.0;
						Output[1] = ga * a * 4.0 * sqrt( surf * sgn * pow( sgn * sin( X ) , alpha ) + 1.0 ) / 10.0 * 0x8000 + 0x8000;
						break;
					case 'f':
						alpha = 0.5;
						Output[1] = ga * a * 4.0 * sqrt( surf * sgn * pow( sgn * sin( X ) , alpha ) + 1.0 ) / 10.0 * 0x8000 + 0x8000;
						break;
					default:
						Output[1] = 0x8000;
				}
			}
			else if( newvalue_f > 20.0 && newvalue_x > -150 && newvalue_x < 150){
				Output[1] = ga * a * 4.0 / 10.0 * 0x8000 + 0x8000;
			}
			else{
				Output[1] = 0x8000;
			}
		}
		else{
			Output[1] = 0x8000;
		}

		if( Output[1]>(6.4/10.0*0x8000+0x8000) ){
			Output[1] = 2.4/10.0*0x8000+0x8000;
		}
		if( Output[1]<(-5.0/10.0*0x8000+0x8000) ){ //0x8000=32768
			Output[1] = -2.4/10.0*0x8000+0x8000;
		}

		//AM変調
		a = -a;

		//kbhit押されていないときは
		BUTTON = 1;

		if (i%100 == 1){
			//printf("%lf %d %d\n ",phyx1, Output[1], i );
			//printf("%lf\n ",phyv1);
			//printf("%lf\n", sgn );
			//printf("%d, %d\n", (int)(ga* basic * 4.0 / 10.0 * 0x8000), (int)(ga * uneve * a * 4.0 * sqrt(sin( (newvalue_x+blen) * 2 * M_PI / (2 * blen) + pha * M_PI) + 1.0) / 10.0 * 0x8000));
			//fprintf(fp1, "%d %lf %d\n",rt_ticks, newvalue_x, Output[1]);
			fprintf(fp1, "%.2lf %.1lf %d\n", newvalue_x, newvalue_f, (int)((Output[1]-0x8000)/20.0) );
			//fprintf(fp1, "%.2lf %lf %lf %lf %d\n", newvalue_x, exp(-pow( EX,2 )/sigma) , (pow(EX,2))/sigma, MAX_EXP, (int)( -2 * EX * exp( -pow( EX,2 ) / sigma) / sigma + MAX_EXP) );
			//printf("MAX_EXP: %.2lf\n", MAX_EXP);
		}
	
		//DA出力
		nRet = DaOutputDA( 1, 2, DaSmplChReq, Output );
			if (nRet) {
				printf("DaOutputDA error: ret=%Xh\n", nRet);
				DaClose(1);
				exit(EXIT_FAILURE);
			}		
		int ticksj = rt_ticks;

		phyvp1 = phyv1; // phyvp is the 1 step previous phyv
	
		rt_waitDA( &rt_ticks );

		if( g_quit ){
			Output[0] = 0x8000;
			Output[1] = 0x8000;			
			nRet = DaOutputDA( 1, 2, DaSmplChReq, Output );
			break;
		}
	}
	c = 'q';
	g_quit = 1;
	DaClose(1);
	printf("press [q] for exit\n");
	return (void *) NULL;
}


void *thread_kbhit(void *thdata){//build kbhit thread
	while(1){
      	       if( kbhit() ){
			c = getchar();
			BUTTON = 0;
		}
      	       if( c == 'q' ){
      	              // Close the device.
			g_quit = 1;
			usleep( 10000 );
			break;
       	}else if( c == 'g' ){//等倍
			Plen = Plen;
			SIN = 0;
			//BUTTON = 1;
		}
		else if( c == 'f' ){//0.7倍
			Plen = Plen;
			SIN = 0;
			//BUTTON = 1;
		}
		else if( c == 'h' ){//1.5倍
			Plen = Plen;
			SIN = 0;
			//BUTTON = 1;
		}
		else if( c == 'b' ){//等倍
		}
		else if( c == 'n' ){//0.7倍
			Plen = Plen;
			SIN = 1;
		}
		else if( c == 'm' ){//1.5倍
			Plen = Plen;
			SIN = 1;
		}
		else if( c == 'v' ){//1.5倍
			Plen = Plen;
			SIN = 1;
		}

      	}
	fprintf( stderr, "end of kbhit\n" );
      	return (void *) NULL;
}

//pthreadによるAD制御
int main( int argc, char *argv[] ){

	int i = 0;

	unsigned long long ticks[TestCount];
	unsigned long long rt_ticks = GetTick();
	system("clear");

	//Open a device.
	ret = AdOpen(1);
	if (ret) {
		printf("AdOpen error: ret=%Xh\n", ret);
		exit(EXIT_FAILURE);
	}	
	ret = AdGetDeviceInfo(1, &BoardSpec);
	if(ret){
		printf("AdGetDeviceInfo error: ret=%Xh\n", ret);
		AdClose(1);
		exit(EXIT_FAILURE);
	}

	// Command line option
	char comop[256];

	printf("argc=%d\n", argc);

	if( argc >= 2 ){
		ga = atof( argv[1] );
		if( ga > 0.8 ){
			ga = 0.8;
		}
	} else{
		ga = 0.4;
	}
	

	if( argc >= 3 ){
		printf("%s\n", argv[2]);
		strcpy( comop, argv[2]);

		// Bump
		if( strcmp( comop, "bump") == 0 ){
			Plen = 20;
			freqv1 = 1.0;
		}
		// Dent
		else if( strcmp( comop, "dent") == 0 ){
			Plen = 20;
			freqv1 = 1.0;
		}
		else{
			printf("Invalid option.\n");
			printf("$ sudo nice -n -20 ./demo [w1, t1] [gain=0.4]\n\n");
			return(0);
		}

	}

	printf("Gain: ga = %f\n", ga);


	//AD Channels
	//condition of sampling
	ret = AdGetSamplingConfig( 1, &AdSmplConfig );
	if (ret) {
		printf("AdGetSamplingConfig error: ret=%Xh\n", ret);
		AdClose(1);
		exit(EXIT_FAILURE);
	}
	//Input Data
	AdSmplConfig.ulChCount = 8;
	AdSmplConfig.SmplChReq[0].ulChNo = 1;
	AdSmplConfig.SmplChReq[0].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[1].ulChNo = 2;	
	AdSmplConfig.SmplChReq[1].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[2].ulChNo = 3;
	AdSmplConfig.SmplChReq[2].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[3].ulChNo = 4;	
	AdSmplConfig.SmplChReq[3].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[4].ulChNo = 5;
	AdSmplConfig.SmplChReq[4].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[5].ulChNo = 6;	
	AdSmplConfig.SmplChReq[5].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[6].ulChNo = 7;
	AdSmplConfig.SmplChReq[6].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplConfig.SmplChReq[7].ulChNo = 8;	
	AdSmplConfig.SmplChReq[7].ulRange = AdSmplConfig.SmplChReq[0].ulRange;

	
	ret = AdSetSamplingConfig(1, &AdSmplConfig);
	if (ret) {
		printf("AdSetSamplingConfig error: ret=%Xh\n", ret);
		AdClose(1);
		exit(EXIT_FAILURE);
	}

	AdSmplChReq[0].ulChNo = 1;
	AdSmplChReq[0].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[1].ulChNo = 2;
	AdSmplChReq[1].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[2].ulChNo = 3;
	AdSmplChReq[2].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[3].ulChNo = 4;
	AdSmplChReq[3].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[4].ulChNo = 5;
	AdSmplChReq[4].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[5].ulChNo = 6;
	AdSmplChReq[5].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[6].ulChNo = 7;
	AdSmplChReq[6].ulRange = AdSmplConfig.SmplChReq[0].ulRange;
	AdSmplChReq[7].ulChNo = 8;
	AdSmplChReq[7].ulRange = AdSmplConfig.SmplChReq[0].ulRange;

	// Determine the offset values of each loadcell	
	double loadCellOffset[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
	for( i=0; i < 50; i++ ){ 
		ret = AdInputAD(1, 8, AD_INPUT_DIFF, AdSmplChReq, Data);
		if (ret) {
			AdClose(1);
			printf("AdInputAD error: ret=%Xh\n", ret);
			exit(EXIT_FAILURE);
		}
		loadCellOffset[0] += ((Data[0]-32767)/6553.4)/50.0;
		loadCellOffset[1] += ((Data[1]-32767)/6553.4)/50.0;
		loadCellOffset[2] += ((Data[2]-32767)/6553.4)/50.0;
		loadCellOffset[3] += ((Data[3]-32767)/6553.4)/50.0;
		loadCellOffset[4] += (double)Data[4]/50.0;
		loadCellOffset[5] += (double)Data[5]/50.0;
		loadCellOffset[6] += (double)Data[6]/50.0;
		loadCellOffset[7] += (double)Data[7]/50.0;

	}
	
	printf("loadCellOffset = [ %lf %lf %lf %lf %lf %lf %lf %lf]\n", loadCellOffset[0], loadCellOffset[1], loadCellOffset[2], loadCellOffset[3],loadCellOffset[4], loadCellOffset[5], loadCellOffset[6], loadCellOffset[7]);
	
	// For saving data
	FILE *fp;
	fp = fopen( "AD.csv", "w" );

	/* For input
	{if (j=0) {
		printf("input number of frequency");
		scanf("%d,&nof");
		switch (nof) {
			case 3;
				while(1){
					c=getchar;
				}
				
		}
		j=j+1;
	}*/

	// For kbhit
	pthread_t pth;
	int rtpth;
	rtpth = pthread_create( &pth, NULL, &thread_kbhit, (void *)NULL );

	
	int DAthr;
	pthread_t dathread;
	DAthr = pthread_create(&dathread, NULL, &Out, (void *)NULL);

	//struct thdata *priv = (struct thdata *)thdata;

	// Retrieve the current input value
	for( i = 0; i < TestCount; i++ ){//i<50 == Testcount


		ret = AdInputAD(1, 8, AD_INPUT_DIFF, AdSmplChReq, Data);
		if (ret) {
			printf("AdInputAD error 1: ret=%Xh\n", ret);
			AdClose(1);
			exit(EXIT_FAILURE);
		}
		

		// unsigned short to double
		Data1[0]=((Data[0]-32767)/6553.4-loadCellOffset[0]) ; //1-z
		Data1[1]=((Data[1]-32767)/6553.4-loadCellOffset[1]) ; // 2-z
		Data1[2]=((Data[2]-32767)/6553.4-loadCellOffset[2]) ; // 3-z
		Data1[3]=((Data[3]-32767)/6553.4-loadCellOffset[3]) ; // 4-z
		Data1[4]=(Data[4]-32767)/6553.4; // 1-x
		Data1[5]=(Data[5]-32767)/6553.4; // 2-y
		Data1[6]=(Data[6]-32767)/6553.4; // 3-y
		Data1[7]=(Data[7]-32767)/6553.4; // 4-z


		if(start==50 && i==49) printf("%lf %lf %lf %lf/n", Data1[0],Data1[1],Data1[2],Data1[3]);

		// voltage to force (N)
		Data2[0]=(0.98/0.288)*(double)Data1[0];// 1-z
		Data2[1]=(0.98/0.238)*(double)Data1[1];// 2-z
		Data2[2]=(0.98/0.294)*(double)Data1[2];// 3-z
		Data2[3]=(0.98/0.226)*(double)Data1[3];// 4-z
		Data2[4]=(0.98/0.44)*(double)Data1[4];// 1-x
		Data2[5]=(0.98/0.4)*(double)Data1[5];// 2-y
		Data2[6]=(0.98/0.52)*(double)Data1[6];// 3-y
		Data2[7]=(0.98/0.44)*(double)Data1[7];// 4-z


		//position
		//int fl = 21; //length of filter
		int k;
		x[0] = 0.0;
		y[0] = 0.0;	

		memset( x_history, 0, sizeof(x_history) );
		memset( y_history, 0, sizeof(y_history) );
		

		newvalue_x = (Data1[0] * (-115.1738) + Data1[1] * (-118.7229) + Data1[2] * 113.978 + Data1[3] * 121.4213) / (Data1[0] + Data1[1] + Data1[2] + Data1[3]);
		newvalue_y = Data1[0] * (-84.9108) + Data1[1] * 94.38 + Data1[2] * 75.2135 + Data1[3] * (-84.9345) / (Data1[0] + Data1[1] + Data1[2] + Data1[3]);
		newvalue_f = Data1[0] * 420.2561 + Data1[1] * 475.5806 + Data1[2] * 410.5575 + Data1[3] * 459.6844;

		//newvalue_x = (-97.5 * (Data2[0] + Data2[1]) + 97.5* (Data2[2] + Data2[3])) / (Data2[0] + Data2[1] + Data2[2] + Data2[3]);
		//newvalue_y =  (-65 * (Data2[1] + Data2[3]) + 65 * (Data2[0] + Data2[2])) / (Data2[0] + Data2[1] + Data2[2] + Data2[3]); 
					
		//limit of position
		/*if (Data2[0] < 0.2 && Data2[1] < 0.2 && Data2[2] < 0.2 && Data2[3] < 0.2){
			newvalue_x = 0;
			newvalue_y = 0;
		}*/

		memmove(&(x_history[1]), x_history, (filn-1) * sizeof(float) );
		x_history[0] = newvalue_x;

		memmove(&(y_history[1]), y_history, (filn-1) * sizeof(float) );
		y_history[0] = newvalue_y;


		for( j = 0, ave_x = 0; j < filn; j++ ){
			ave_x += x_history[j];
			ave_y += y_history[j];
		}

		ave_x = (ave_x / (double)filn) ; //unit is mm
		ave_y = (ave_y / (double)filn) ; //unit is mm

		dx = ave_x - ave_xp;  // dx is the difference of finger position in x-direction  :the unit is mm  
		dy = ave_y - ave_yp;

		dxa = fabs(dx); // dxa is the absolute value of dx :unit is mm
		dya = fabs(dy); // dxa is the absolute value of dx :unit is mm		
		
		dv = sqrt(pow(dxa,2) + pow(dya,2));

		ave_xp = ave_x;  // ave_xp is the previous finger position in x-direction 
		ave_yp = ave_y;  // ave_xp is the previous finger position in y-direction 

		//dv = sqrt(pow(dxa,2) + pow(dya,2))*(1000/rt_cycleAD); // translate into mm/s
		

		if (dv > 300){
			dv =300;
		}

		if( i % 3 == 0 ){
			//printf(fp,"%d \n ",rt_ticks); 
			//printf("%d \n ",rt_ticks); 
			//printf("%lf %lf %lf %lf\n",Data2[0],Data2[1],Data2[2],Data2[3]);
			//printf("%lf %lf %lf %lf\n",Data2[4],Data2[5],Data2[6],Data2[7]);
			//fprintf(fp,"%lf %lf\n",sf,mup);
			//printf("%lf\n",dv);
			//printf("%lf %lf\n",nf,dv);
			printf("%.2lf mm, %.2lf mm, %.1lf gw, kbhit: %c\n",newvalue_x, newvalue_y, newvalue_f, c);
			//printf("%lf\n",dv);
			//fprintf(fp,"%lf %lf %lf\n",nf,sf,mup);
			//printf("%lf %lf %lf %lf %lf %lf\n",Data1[0],Data1[1],Data1[2],Data1[3],newvalue_x,newvalue_y);
			//printf("%lf %lf\n",dxa,dya);
		}

		rt_waitAD( &rt_ticks );
		ticks[i] = rt_ticks;

		if( g_quit ){
			break;
		}

               }

	printf("End\n");
	// Close the device.
	//pthread_exit(&DAthr);	
	//pthread_exit(&rtpth);	
	//ret = pthread_join(dathread,NULL);
	AdClose(1);
	DaClose(1);
	fclose( fp );
	printf("Main is closed\n");
	return 0;
	
}
