/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
/*******************************************************************************
 *  INCLUDE #define POLYNORMIAL 0xA001FILES
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>

//multi thread
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define _USE_MATH_DEFINES

typedef unsigned char BYTE;

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

union
{
    float data ;
    char  bytedata[4];
    
} m_robot_speed;

union
{
    short data ;
    char  bytedata[2];
    
} m_robot_angle, m_robot_angle1;

#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 



#define BAUDRATE B115200
#define SERIAL_DEVICE "/dev/ttyUSB2"  
#define PACKET_LEN 8
#define BUF_SIZE 20

static int uart_fd;
//unsigned char protocal_test[13] ={0,};
unsigned char protocal_receive[BUF_SIZE] = {0,};

int cnt_rcv = 0;
short check = 0;

void write_serial(unsigned char *buf, int len)
{
   write(uart_fd, &buf[0], len);
} 


int init_serial_port(void)
{
  int serial_port = open(SERIAL_DEVICE, O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 100;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, BAUDRATE);
  cfsetospeed(&tty, BAUDRATE);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return -1;
  }
    
  else
  {
      return serial_port;
  } 
}

void *readserial_thread(void *pt)
{
    int num_bytes = -1;

    unsigned char insert_buf; 
    
   
    while(1)
    { 
        while( (num_bytes = read(uart_fd, &insert_buf, 1)   ) > 0 )   
        {
            protocal_receive[cnt_rcv] = insert_buf;
            printf("%c %d \n", protocal_receive[cnt_rcv], protocal_receive[cnt_rcv]);

            if(protocal_receive[cnt_rcv] == '*')
            {
				cnt_rcv = (cnt_rcv - PACKET_LEN + BUF_SIZE)%20;
                    
                //printf("\n\n\n%c\n\n\n", protocal_receive[cnt_rcv]);
                
                // cnt_rcv = (cnt_rcv - PACKET_LEN + BUF_SIZE)%BUF_SIZE; -> 한 줄로 음수, 양수 상관없이 양수로 만들 수 있음 (조건문이 필요없다 이말이야)

                if(protocal_receive[cnt_rcv] == '#' && (protocal_receive[(cnt_rcv+1)%BUF_SIZE] == 'I' || protocal_receive[(cnt_rcv+1)%BUF_SIZE] == 'F' ))
                {
					m_robot_angle.data = 0;
					
                    for(int i = 1 ; i < 6 ; i++)
                    {
                        m_robot_angle.data += protocal_receive[(cnt_rcv+i)%BUF_SIZE];
                    }

                    if(protocal_receive[(cnt_rcv + 6)%BUF_SIZE] == m_robot_angle.bytedata[1] && protocal_receive[(cnt_rcv + 7)%BUF_SIZE] == m_robot_angle.bytedata[0])
                    {
                        m_robot_angle1.bytedata[1] = protocal_receive[(cnt_rcv + 2)%BUF_SIZE];
                        m_robot_angle1.bytedata[0] = protocal_receive[(cnt_rcv + 3)%BUF_SIZE];

                        printf("success! %d\n\n",m_robot_angle1.data);

                    }
                    
                    else
                    {
                        printf("crc error\n");

                        printf("MSB : %x, LSB : %x\n\n", m_robot_angle.bytedata[1], m_robot_angle.bytedata[0]);

                    }

                }
            }
        cnt_rcv++;
        cnt_rcv %= BUF_SIZE;    

            
    /*
       for(int i=0;i<10;i++)
            {
                read_buf[i]=read_buf[i+1];
            }
            read_buf[10]=insert_buf;
    */   
      // for(int i=0;i<10;i++)       printf("%c", read_buf[i]);
      // printf("\n");
        }
    }
}  


int main(void)
{

  uart_fd = init_serial_port();  
   
  
  pthread_t id_1;
  
  int ret1=pthread_create(&id_1,NULL,readserial_thread,NULL);    
        
  while(1)
  {

  }

  close(uart_fd);
  return 0;
  
}
