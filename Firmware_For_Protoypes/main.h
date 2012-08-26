#include <18F458.h>
#device adc=8
#fuses H4,NOWDT,WDT32,NOPROTECT,NOLVP,PUT,BROWNOUT,BORV27


#use delay(clock=40000000)
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)

