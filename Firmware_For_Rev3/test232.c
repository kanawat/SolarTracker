#include <18F458.h>
#fuses H4,NOWDT,WDT128,NOPROTECT,NOLVP,PUT,NOBROWNOUT
#use fixed_io(d_outputs=PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7)
#use delay(clock=40000000)
#use rs232(baud=9600, xmit=PIN_C6, rcv=PIN_C7, BRGH1OK)

void main() {
	while(1) {
		printf("\r\n hello!");
		delay_ms(500);
	}
}

