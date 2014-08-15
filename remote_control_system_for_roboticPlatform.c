#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <rbio/roboard.h>

#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include <strings.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include  <sys/wait.h>
#include <fcntl.h>
#include <time.h>

using namespace std;

// wheel solenoid parameters
const int PULSE_WIDTH = 20000;
const int PULSE_COUNT = 20;
const int MOTOR_DELAY = 10000;
const int REVERSAL_DELAY = 300000;
const int IDLE_SPEED = 1535;
const int IMPULSE = 100;
const double SPEED_MULTIPLIER = 90000.0;
const double BUMP_SIZE = 7000.0;
const int ROTATION_SPEED = 2;    //speed factor for automatic rotation of robot

// socket communication parameters
const int SOCKET_PORT = 3490;
const int BUFFSIZE = 32;
const char IMU_FILE_NAME0[] = "/dev/ttyUSB0";
const char IMU_FILE_NAME1[] = "/dev/ttyUSB1";

// shared memory parameters
const int SHMKEY_LEFT = 10000;
const int SHMKEY_RIGHT = 20000;
const int SHMKEY_IMU = 30000;

// IMU parameters
const double NORTH = 180.0;
const double SOUTH = 0.0;
const double EAST = 90.0;
const double WEST = -90.0;
const double HEADING_EPSILON = 10.0;


enum {KILL, FWD, BACK, LEFT, RIGHT, AUTO, EXIT};


// shared memory variables
int shmid_left = shmget( SHMKEY_LEFT, sizeof(int), 0666|IPC_CREAT );
int shmid_right = shmget( SHMKEY_RIGHT, sizeof(int), 0666|IPC_CREAT );
int shmid_imu = shmget( SHMKEY_IMU, sizeof(double), 0666|IPC_CREAT );
int* left_duty = (int*)shmat( shmid_left, NULL, 0 );
int* right_duty = (int*)shmat( shmid_right, NULL, 0 );
double* imu_data = (double*)shmat( shmid_imu, NULL, 0 );


// servo control functions
void kill()
{
    printf("\nKILL signal received\n");
*left_duty = IDLE_SPEED;
*right_duty = IDLE_SPEED;
}

void forward()
{
    printf("\nFORWARD signal received\n");
*left_duty += IMPULSE;
*right_duty -= IMPULSE;
}

void reverse()
{
    printf("\nBACK signal received\n");
*left_duty -= IMPULSE;
*right_duty += IMPULSE;
}

void left()
{
    printf("\nLEFT signal received\n");
*left_duty -= IMPULSE;
*right_duty -= IMPULSE;
}

void right()
{
    printf("\nRIGHT signal received\n");
*left_duty += IMPULSE;
*right_duty += IMPULSE;
}


// provides autonomous straight line sequence
void straightLeg( int distInCms, int speed )
{
    // stop motors
    kill();
// calculate travel time
if( speed == 0 ) return;
    double travelTime = distInCms / speed;
// set duty cycles per desired speed
    *left_duty = IDLE_SPEED + speed * IMPULSE;
    *right_duty = IDLE_SPEED - speed * IMPULSE;
// sleep until distance covered
usleep( travelTime * SPEED_MULTIPLIER );
// stop motors
    kill();
}


// sample the IMU input stream
double getIMUnum( ifstream& imu )
{
    double imuHeading = 0.0;
char imuBuffer[32] = {0};
char streamChar = 0;
int bufferPos = 0;
bool foundNumber = false;
bool startNumber = false;

while ( !foundNumber )
{
imu >> streamChar;
if( streamChar == '~' ) // end number flag
{
   foundNumber = true;
}
else
{
   if( startNumber )
       imuBuffer[bufferPos++] = streamChar;
   if( streamChar == '!' ) // start number flag
       startNumber = true;
   }
    }
// convert ascii data from input stream to a floating-point value
imuHeading = atof( imuBuffer );
// avoid a "race condition" during thread termination
if(*imu_data == -3444)
   return imuHeading;
// store the parameter in shared memory
*imu_data = imuHeading;
return imuHeading;
}


// gets heading data from IMU
double getHeading()
{
    double imuDiff = 500.0;
double imu1 = 0.0;
double imu2 = 0.0;
// get two headings and test for stability
while( imuDiff > 0.1 )
{
imu1 = *imu_data;
usleep(25000);
imu2 = *imu_data;
imuDiff = fabs( imu1 - imu2 );
}
// for testing
printf( "imu2 = %lf\n", imu2 );
return imu2;
}


// rotates the robot counterclockwise by a small amount
void bumpHeadingLeft( double bumpSize, int rotationSpeed )
{
    // stop motors
kill();
// rotate left
*left_duty = IDLE_SPEED - rotationSpeed * IMPULSE;
*right_duty = IDLE_SPEED - rotationSpeed * IMPULSE;
// sleep per bump size
usleep( bumpSize );
// stop motors
kill();
}


// provides autonomous rotation sequence
void orient( double direction )
{
    double imuHeading = getHeading();
// rotate left until close to desired heading
    while( fabs( direction - imuHeading ) > HEADING_EPSILON )
{
   bumpHeadingLeft( BUMP_SIZE, ROTATION_SPEED );
   imuHeading = getHeading();
}
}


// provides autonomous navigation of a square
void autoSquare()
{
int pid = fork();
if( pid != 0 )
{
   // report thread startup
cout << "imu reader: run." << endl;
// try primary file name for IMU data stream
ifstream imu( IMU_FILE_NAME0 );
if( !imu ) // try alternate file name for IMU data stream
{
imu.open( IMU_FILE_NAME1 );
}
if( !imu ) 
{
printf("\nUnable to open imu file\n");
return;
}

// have this child thread continuously sample the IMU
while(1)
{
if(*imu_data == -3444)
{
*imu_data = 0;
imu.close();
cout << "imu reader: exit." << endl;
exit(0);
}
getIMUnum(imu);
}
}
    int dist = 300;  // in centimeters
int speed = 2;  // number of impulse "bumps"
    printf("\nAUTO signal received\n");
    // stop
kill();
//sleep to let the IMU init
sleep(3);
// point North
orient( NORTH );
// go fwd 3 mtrs
straightLeg( dist, speed );
// point West
orient( WEST );
// go fwd 3 mtrs
straightLeg( dist, speed );
// point South
orient( SOUTH );
// go fwd 3 mtrs
straightLeg( dist, speed );
// point East
orient( EAST );
// go fwd 3 mtrs
straightLeg( dist, speed );
// stop
kill();

// signal IMU monitoring thread to exit
*imu_data = -3444;
printf("\nAUTO sequence completed\n");
}


// a continuously-running thread to control the robot servos
void runServos()
{
    while( 1 ) {
   // terminate thread when parent signals completion
   if(*right_duty == -2550) exit(0);
if(*right_duty != IDLE_SPEED && *left_duty != IDLE_SPEED) {
// right wheel servo
rcservo_SendPWMPulses(0,PULSE_WIDTH,*right_duty,PULSE_COUNT);
// left wheel servo
rcservo_SendPWMPulses(1,PULSE_WIDTH,*left_duty,PULSE_COUNT);
while ( rcservo_IsPWMCompleted(1) == false ) {}

}
}
}


// respond to error condition
void error(char *msg)
{
    // report error
    perror(msg);
// exit program with error code
    exit(1);
}


// respond to client navigation commands
int HandleClient(int sock) {
    char buffer[BUFFSIZE];
    int received = -1;
int command = 0;
    /* Receive message */
    if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
       error("Failed to receive initial bytes from client");
    }
// spawn child process to run servos continuously
int pid = fork();
if( pid != 0 ) runServos();

    /* Send bytes and check for more incoming data in loop */
    while (received > 0) {
      /* Send back received data */
      if (send(sock, buffer, received, 0) != received) {
        error("Failed to send bytes to client");
      }
 
 // respond to command
 command = atoi( buffer );
 switch( command )
      {
         case FWD:    forward(); break;
         case BACK:   reverse(); break;
         case RIGHT:  right(); break;
         case LEFT:   left(); break;
    case AUTO:   autoSquare(); break;
    case EXIT:
    case KILL:   kill(); break;
      }
 
      /* Check for more data */
      if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
        error("Failed to receive additional bytes from client");
      }
    }
// close the network socket when the client leaves
    close(sock);
return command;
}



int main(void) {

    printf("\nStarting Control Program\n");

int status;
*left_duty = IDLE_SPEED;
*right_duty = IDLE_SPEED;
rcservo_Initialize(3);
rcservo_EnterPWMMode();
// variables to support a network socket
int sockfd, newsockfd, portno;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
int command;
     
// setup a network socket and listen for the remote client
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = SOCKET_PORT;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
 
    listen(sockfd,5);
     
 
// connect to the client and enter a listening function
while( command != EXIT ) {
   socklen_t clilen = sizeof(cli_addr);
   newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr, &clilen);
        if (newsockfd < 0) 
         error("ERROR on accept");
 
   printf("\nconnected to client\n");
command = HandleClient(newsockfd);
printf("\ndisconnected from client\n"); 
}
rcservo_Close();
// set exit variable for child process and wait for child to exit
*right_duty = -2550;
wait(&status);
wait(&status);
printf("\nExiting Control Program\n");
return 0;
}