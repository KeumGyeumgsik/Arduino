#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"

#define LED_GPIO_DIR "/sys/class/gpio"

int main(){
 int  port_num = 216, value;
 char buff[256]; //buff(버퍼)란 데이터를 한 곳에서 다른 한 곳 으로 전송하는 동안 일시적으로 그 데이터를 보관하는 메모리의 영역이다.
 FILE *fp;       //buggering(버퍼링)이란 버퍼를 활용하는 방식 또는 버퍼를 채우는 동작을 말한다. -> Queue(큐)라고도 한다.

 snprintf(buff,sizeof(buff),"%s/export",LED_GPIO_DIR);
 /* 
 snprintf => 형식화된 데이터를 버퍼로 출력 / 패킷 통신을 하거나 buffer에 원하는 문자열을 삽입, 이어붙이기 할때 자주 사용되는 함수이다.
 int snprintf ( char * s, size_t n, const char * format, ... );
 첫번째 인자에 문자열을 저장할 배열을 넘겨준다. 두번째 인자는 저장할 배열의 크기, format에는 ""로 묶여진 서식지정자와 문자를 주면 버퍼
 에 해당내용이 저장, 리턴값으로는 buffer에 저장한 배열의 크기를 반환하게 된다.
 sprintf보다 snprintf를 더 많이 사용한다.
 */
 fp = fopen(buff,"w");
 fprintf(fp,"%d\n",port_num); //fprintf => 스트림에 형식화된 데이터 쓰기 / 파일에 문자열 쓰기.
 /*
 fprintf(파일포인터, 서식, 값1, 값2, ...);
 int fprintf(FILE * const _Stream, char const * const _Format, ...);
 성공하면 쓴 문자열의 길이를 반환, 실패하면 음수를 반환
 */
 fclose(fp);

 snprintf(buff,sizeof(buff),"%s/gpio%d/direction",LED_GPIO_DIR,port_num);
 fp = fopen(buff,"w");
 fprintf(fp,"out\n");
 fclose(fp);

 snprintf(buff,sizeof(buff),"%s/gpio%d/value",LED_GPIO_DIR,port_num);
 fp = fopen(buff,"w");
 setvbuf(fp, NULL, _IONBF, 0);
  
 value = 0;
 for(int i=0;i<=10;i++){
  fprintf(fp,"%d\n",value);
  sleep(1);
  value = !value;
 }
 fclose(fp);
 // buffer overflow(버퍼 오버플로)란 메모리를 다루는 데에 오류가 발생하여 잘못된 동작을 하는 프로그램 취약점이다.
 // 받아오는 크기만큼 복사를 하여 buf의 size보다 큰 문자열을 받게된다면 오버플로우가 발생한다.

 snprintf(buff,sizeof(buff),"%s/unexport",LED_GPIO_DIR);
 fp = fopen(buff,"w");
 fprintf(fp,"%d\n",port_num);
 fclose(fp);

 return 0;
}
