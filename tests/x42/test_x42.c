#include "x42.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t input[80], sent[8];
static uint16_t count, pos, sent_len;
static uint32_t tick, calls, byte_delay;
static HAL_StatusTypeDef tx_status, end_status;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n, uint32_t t)
{ (void)u; assert(t == 100U); assert(n <= sizeof(sent)); memcpy(sent,p,n); sent_len=n; calls++; return tx_status; }
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *u, uint8_t *p, uint16_t n, uint32_t t)
{
 (void)u; assert(n==1); assert(t>0 && t<=50);
 if(pos<count) { if(byte_delay>t) { tick+=t; return HAL_TIMEOUT; } *p=input[pos++]; tick+=byte_delay; return HAL_OK; }
 tick+=t; return end_status;
}
static void setup(const uint8_t *p, uint16_t n)
{ if(n) memcpy(input,p,n); count=n; pos=0; tick=0; calls=0; sent_len=0; byte_delay=0; tx_status=HAL_OK; end_status=HAL_TIMEOUT; }
int main(void)
{
 UART_HandleTypeDef uart={HAL_UART_STATE_READY,HAL_UART_STATE_READY}; X42_HandleTypeDef motor;
 const uint8_t ack[]={1,0xf3,2,0x6b}, run[]={1,0xf6,2,0x6b};
 const uint8_t raw[]={1,0x35,0x6b,0,0xff,0x6b}; uint16_t n;
 struct { uint8_t before, data[6], after; } b={0xa5,{0},0x5a};
 X42_Init(&motor,&uart,1); X42_Init(NULL,&uart,1);
 setup(ack,4); assert(X42_Enable(&motor)==X42_OK); assert(sent_len==4 && !memcmp(sent,ack,4));
 setup(run,4); assert(X42_Run(&motor)==X42_OK); assert(!memcmp(sent,run,4));
 for(unsigned i=0;i<4;i++) { setup(ack,4); input[i]^=1; assert(X42_Enable(&motor)==X42_BAD_RESPONSE); }
 setup(ack,3); assert(X42_Enable(&motor)==X42_BAD_RESPONSE);
 setup(ack,4); input[4]=0; count=5; assert(X42_Enable(&motor)==X42_BAD_RESPONSE);
 setup(NULL,0); assert(X42_Enable(&motor)==X42_TIMEOUT);
 setup(NULL,0); tx_status=HAL_ERROR; assert(X42_Enable(&motor)==X42_ERROR && pos==0);
 setup(NULL,0); tx_status=HAL_TIMEOUT; assert(X42_Enable(&motor)==X42_TIMEOUT);
 setup(ack,2); end_status=HAL_ERROR; assert(X42_QuerySpeed(&motor,b.data,6,&n)==X42_ERROR && n==2);
 setup(raw,6); assert(X42_QuerySpeed(&motor,b.data,6,&n)==X42_OK && n==6); assert(!memcmp(b.data,raw,6)); assert(sent_len==3 && sent[1]==0x35);
 setup(raw,6); assert(X42_QueryStatus(&motor,b.data,5,&n)==X42_BUFFER_TOO_SMALL && n==5); assert(sent[1]==0x3a); assert(b.before==0xa5 && b.after==0x5a);
 setup(raw,6); assert(X42_QuerySpeed(&motor,b.data,0,&n)==X42_ERROR && n==0 && calls==0);
 n=99; assert(X42_QueryStatus(NULL,b.data,6,&n)==X42_ERROR && n==0);
 assert(X42_QuerySpeed(&motor,NULL,6,&n)==X42_ERROR); assert(X42_QuerySpeed(&motor,b.data,6,NULL)==X42_ERROR);
 assert(X42_Enable(NULL)==X42_ERROR); assert(X42_Stop(&motor)==X42_NOT_SUPPORTED && calls==0);
 uart.RxState=0; assert(X42_Enable(&motor)==X42_ERROR && calls==0); uart.RxState=HAL_UART_STATE_READY;
 setup(ack,4); tick=UINT32_MAX-1; byte_delay=1; assert(X42_Enable(&motor)==X42_OK);
 setup(NULL,0); memset(input,0x6b,sizeof(input)); count=80; byte_delay=2;
 { uint8_t large[80]; assert(X42_QuerySpeed(&motor,large,80,&n)==X42_TIMEOUT && n==25 && tick==50); }
 puts("X42 mock HAL tests: PASS"); return 0;
}
