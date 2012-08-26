#include <18F458.h>
#device ADC=10
#include <stdlib.h>
//#include <math.h>

#fuses HS,NOWDT,WDT128,NOPROTECT,NOLVP,PUT,NOBROWNOUT
#use fixed_io(d_outputs=PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7)
#use delay(clock=10000000)
#use rs232(baud=38400, xmit=PIN_C6, rcv=PIN_C7, BRGH1OK, parity =N,BITS=8)
#use spi(MASTER, DI=PIN_C4, DO=PIN_C5, CLK=PIN_C3, BITS=8, MSB_FIRST, IDLE=0)




/*
struct port_d_map{
   int8 data_bus:4; //0:3
   boolean rs;     //4
   boolean LED_latch;  //5
   boolean MUX_en;  //6
   boolean disp_en;   //7
} PORTD;
#byte PORTD =0xF83
*/
#byte SSPSTAT = 0xFC7
#byte SSPCON1 = 0xFC6
#byte SSPCON2 = 0xFC5
#byte PIE1 = 0xF9D
#byte RCSTA= 0xFAB
#byte TXSTA= 0xFAC
#byte PIR1 = 0xF9E
#byte TMR1H = 0xFCF
#byte TMR1  = 0xFCE
#byte T1CON = 0xFCD

////////////// various constant /////////
#define Y2010_UNIX_TIME 1262304000
#define Y2010_JDN 2455197.5
#define UTC 7
#define SEC_IN_4_YEARS 126230400
#define MAX_CMD_LEN 18
#define TX_DLY_TIME 1
/////////////////  I/O ////////////
#define EN0 PIN_B0
#define EN1 PIN_B1
#define EN2 PIN_B2
#define EN3 PIN_B3
#define CCW PIN_B4
#define FLASH_CS PIN_C2
//#define TX_EN PIN_E2
#define SENSE_0     PIN_A1
#define SENSE_1     PIN_A2
#define SENSE_2     PIN_A3
#define SENSE_3     PIN_A4
#define WALL_PWR    PIN_A5 //input
///////////// DATA STORAGE ////////
#define ADDR_FULL_STROKE    0xf00000
#define ADDR_CURRENT_STROKE 0xf00010
#define ADDR_TIME           0xf00020
#define ADDR_START_COUNTER  0xf00030
#define MAX_FULL_STROKE 2000
#rom ADDR_FULL_STROKE={0x047E,0x037F,0x0280,0x0181} // default value = 0x2D8 = 728 tick
#rom ADDR_CURRENT_STROKE={0x0000,0x0001,0x0002,0x0003}
#rom ADDR_TIME={0x0000,0x0000,0x0001,0x0000}
#rom ADDR_START_COUNTER={0x0000}
#rom 0xf00040 ={0x1234} // to check if eeprom can be read properly
#define LOGGER_START_PAGE 1500
//////////////////////////////////////


/////////////////////////// flag
struct flag {
   boolean task1_armed;
   boolean update_time;
   boolean prev_pulse_state;
   boolean cmd_posted;
   boolean setup_required;
   boolean reset_rs232;
   boolean pwr_state;
   boolean measure_current;
} flag;

struct flag2 {
   boolean en_operate;
   boolean button_pressed;
   boolean abort_current_activity;
   boolean allow_manual_move_act;
   boolean current_pulse_state;
   boolean power;
   boolean is_moving;
   boolean unused3;
} flag2;
/*
struct LED_status {
  boolean power;
  boolean operation;
  int8 aux:2;
  int8 unused:4;
} LED_status;
*/
struct port_d_map{
   int8 lcd_nibble:4; //0:3
   boolean ps_en;     //4
   boolean meas_rly;  //5
   boolean pwr_ctrl;  //6
   boolean disp_en;   //7
} PORTD;
#byte PORTD =0xF83

struct port_e_map{
   boolean rs; //0
   boolean emgcy; //1
   boolean tx_en; //2
   int8 unused:5;
} PORTE;
#byte PORTE =0xF84

/////////////////////////// timer_related
unsigned int32 timer_sec =0;
unsigned int32 last_command =0;
unsigned int32 tick =0;
unsigned int32 tick2 =0;
unsigned int32 nDay=1;
unsigned int16 startup_counter=0;
/////////////////////////// command processing
char cmd_msg[20];
int8 cmd_len=0;
int8 nButton=-1;

/////////////////////////// string format
char tmp_str[20];
char tmp_str2[20];
const int8 line[4] = {0,0x40,0x10,0x50};
//////////////////////////// ADC
int16 wall_pwr_read;
int16 accumulate_current=0;
int8  n_avg=0;
/////////////////////////// flash_related_variable
int8 flash_mfg_id[4];
int8 flash_stat=0;
int8 flash_page_data=0;
int8 flash_page_data2=0;
/////////////////////////// actuator related
int16 move_act_time_out=3;
int16 actuator_pulse=0;
float act_len;
unsigned int16 current_act_position[4]={0,0,0,0};
unsigned int16 target_act_position=0;
unsigned int16 act_full_stroke_tick[4]={0,0,0,0};
unsigned int16 act_max_stroke=0;
unsigned int16 act_min_stroke=0;
unsigned int16 act_safety_stroke=0;
unsigned int16 latitude=0;
unsigned int16 longitude=0;
unsigned int16 altitude=0;
int8 actuator_move_mask=0x00;
/////////////////// LCD FUNCTION
void lcd_send_nibble( BYTE n ) {
      PORTD.lcd_nibble=n>>4;
      delay_us(50);
      PORTD.disp_en=1;
      delay_us(50);
      PORTD.disp_en=0;
     delay_us(50);
}

void lcd_send_cmd(boolean rs, int8 dat) {
   PORTE.rs=rs;
   lcd_send_nibble(dat&0xF0);
   lcd_send_nibble((dat<<4)&0xF0);
   PORTE.rs=1;
}

void lcd_init() {
   int8 i;
    PORTE.rs = 0;
    PORTD.disp_en=0;
   delay_ms(15);
    for(i=1;i<=3;++i) {
       lcd_send_nibble(0x30); 
       delay_ms(15);
    }
    lcd_send_nibble(0x20); 
    delay_ms(15);
    lcd_send_cmd(0,0x06);
    delay_us(50);
    lcd_send_cmd(0,0x0c);
    delay_us(50);
    lcd_send_cmd(0,0x10);
    delay_us(50);
    lcd_send_cmd(0,0x2c); // 8bit data, 2lines, 5x8 font
    delay_us(50);
    lcd_send_cmd(0,0x01);
    delay_us(50);

}

void lcd_gotoxy( BYTE x, BYTE y) {
   BYTE address;
   address=line[y%4];
   address+=x;
   lcd_send_cmd(0,0x80|address);
}

void lcd_putc( char c) {
   switch (c) {
     case '\f'   : lcd_send_cmd(0,1);
                   delay_ms(2);
                                           break;
     case '\n'   : lcd_gotoxy(1,2);        break;
     case '\b'   : lcd_send_cmd(0,0x10);  break;
     default     : lcd_send_cmd(1,c);     break;
   }
}

void lcd_put_str(char* c, int8 size)
{
   int8 i;
   for(i=0;i<size;i++)   lcd_putc(c[i]);
}



//////////////////////////////////////////////////////
void button_scan();

/////////////////////////////////////////////////////

void init_rs232() {
   bit_clear(PIR1,4);  //TXIF=0
   bit_clear(PIR1,5);  //RCIF=0
   bit_clear(PIE1,5);  //RCIE=0
   bit_clear(RCSTA,7); //SPEN=0
   bit_clear(RCSTA,4); //CREN=0
    bit_clear(TXSTA,4); //SYNC=0
    bit_clear(TXSTA,5); //TXEN=0
   delay_cycles(10);
   bit_set(RCSTA,4); //CREN=1
   bit_set(RCSTA,7); //SPEN=1
    bit_set(TXSTA,5); //TXEN=1
   bit_set(PIE1,5); //RCIE=1

}


void flash_read_mfg_id() {
		output_low(FLASH_CS);
		delay_cycles(20);
		spi_write(0x9F);
		flash_mfg_id[0] = spi_read(0);
		flash_mfg_id[1] = spi_read(0);
		flash_mfg_id[2] = spi_read(0);
		flash_mfg_id[3] = spi_read(0);
		delay_cycles(20);
		output_high(FLASH_CS);
}

void flash_read_stat() {
		output_low(FLASH_CS);
		delay_cycles(20);
		spi_write(0xD7);
		flash_stat = spi_read(0);
		delay_cycles(20);
		output_high(FLASH_CS);
}

void flash_wait_until_ready() {
   int8 wait_loop=0;
   for ( wait_loop=0; wait_loop<0xFF; wait_loop++) {
	 flash_read_stat();
     if ((flash_stat & 0xBF)==0x9C) break;
	 delay_cycles(100);
   }
}

void flash_read_page(int16 pageAddress, int8 pageIndex) {

	//pageAddress <<= 1;
	//pageAddress &= 0xFE;
	flash_wait_until_ready();
   	output_low(FLASH_CS);
	delay_cycles(20);
   	spi_write(0xD2);
    spi_write(make8(pageAddress,1));
    spi_write(make8(pageAddress,0));
    spi_write(pageIndex);
    spi_write(0);
    spi_write(0);
    spi_write(0);
    spi_write(0);
	flash_page_data = spi_read(0);
	flash_page_data2 = spi_read(0);
   	output_high(FLASH_CS);
}

void flash_block_erase() {
   int8 i,j;
   i=0xFF;
   do
   {
   i++;
   flash_wait_until_ready();
   output_low(FLASH_CS);
   spi_write(0x50);
   j = i>>5;
   spi_write(j);
   j = i<<3;
   spi_write(j);
   spi_write(0);
   output_high(FLASH_CS);
   } while(i!=0xFF);
   flash_wait_until_ready();
}

void flash_buffer1_write(int8 data, int8 PageIndex, int8 nData) {
   int i;
   flash_wait_until_ready();
   output_low(FLASH_CS);
   delay_cycles(20);
   spi_write(0x84);
   spi_write(0);
   spi_write(0);
   spi_write(PageIndex);
   if (nData>1) {
   i=255;
   do {
      spi_write(data);
      i++;
   } while(i<nData);
   } else
         spi_write(data);
   output_high(FLASH_CS);
}

void flash_buffer1_read(int8 PageIndex) {
   int i;
   flash_wait_until_ready();
   output_low(FLASH_CS);
   delay_cycles(20);
   spi_write(0xD1);
   spi_write(0);
   spi_write(0);
   spi_write(PageIndex);
   flash_page_data = spi_read(0);
   flash_page_data2 = spi_read(0);
   output_high(FLASH_CS);   
}

void flash_set_256_page_size() {
   flash_wait_until_ready();
   output_low(FLASH_CS);
   delay_cycles(20);
   spi_write(0x3D);
   spi_write(0x2A);
   spi_write(0x80);
   spi_write(0xA6);
   output_high(FLASH_CS); 
}

void flash_write_buffer1_to_main_memory(int16 pageAddress) {
   //pageAddress = pageAddress <<1;
   //pageAddress &= 0xFE;
   flash_wait_until_ready();
   output_low(FLASH_CS);
   delay_cycles(20);	
   spi_write(0x83);
   spi_write(make8(pageAddress,1));
   spi_write(make8(pageAddress,0));
   spi_write(0);
   output_high(FLASH_CS);
}

void flash_write_page(int16 pageAddress) {
   int8 i,check_sum;
   char input_data;
   disable_interrupts(GLOBAL);
   output_high(FLASH_CS);
   flash_wait_until_ready();
   output_low(FLASH_CS);
   if (pageAddress%2) spi_write(0x82); else spi_write(0x85);
   //pageAddress = pageAddress <<1;
   //pageAddress &= 0xFE;
   spi_write(make8(pageAddress,1));
   spi_write(make8(pageAddress,0));
   spi_write(0);
   check_sum=0;
   i=0xFF;
   check_sum=0xCC;
   do {
      input_data = getc();
      check_sum ^= input_data;
      spi_write(input_data);
      i++;
   } while(i!=0xFF);
   output_high(FLASH_CS);
   enable_interrupts(GLOBAL);
}

void flash_read_main_memory_to_buffer1(int16 pageAddress) {
   flash_wait_until_ready();
   output_low(FLASH_CS);
   spi_write(0x53);
   spi_write(make8(pageAddress,1));
   spi_write(make8(pageAddress,0));
   spi_write(0);
   output_high(FLASH_CS);
}


void print_date_time(int8 output_stream) {

	  if(bit_test(output_stream,0))printf("\r\n Day#%lu, ",nDay);
	  
	  if(bit_test(output_stream,1)) {
       lcd_gotoxy(0,0);
	   lcd_putc(0);
       lcd_gotoxy(4,0);
	   printf(lcd_putc,"Day#%4lu, ",nDay );
	  }
      strcpy(tmp_str,"00:00:00");
      itoa((int8) (timer_sec/3600),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+2-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      itoa((int8) ((timer_sec%3600)/60),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+5-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      itoa((int8) (timer_sec%60),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+8-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
	  lcd_gotoxy(4,1);
	  if(bit_test(output_stream,0)) printf("%s",tmp_str);
	  if(bit_test(output_stream,1)) printf(lcd_putc,"%s",tmp_str);
	  lcd_gotoxy(4,2);
	  if(bit_test(output_stream,1)) printf(lcd_putc,"Rst:%4lu",startup_counter);

}

void print_page_data(int16 nPage) {
    int8 i;
    int8 j;
    for(i=0;i<16;i++) {
      printf("\r\n%02X : ",i);
      for (j=0;j<8;j++) {
        flash_read_page(nPage,i*16+j*2);
		printf("%02X %02X ",flash_page_data,flash_page_data2);
	  }
   }
}

void solar_load_parameter_from_flash() {
    flash_read_page(0,0x4E); 
	act_min_stroke = make16(flash_page_data2,flash_page_data);
    flash_read_page(0,0x50); 
	act_max_stroke = make16(flash_page_data2,flash_page_data);
    flash_read_page(0,0x56); 
	latitude = make16(flash_page_data2,flash_page_data);
    flash_read_page(0,0x58); 
	longitude = make16(flash_page_data2,flash_page_data);
    flash_read_page(0,0x5A); 
	altitude = make16(flash_page_data2,flash_page_data);	
    flash_read_page(0,0x5C); 
	act_safety_stroke = make16(flash_page_data2,flash_page_data);
}

void print_fw_info() {
    int8 i,j;
	disable_interrupts(GLOBAL);
    for (i=0;i<4;i++) {
        printf("\r\n");
		for (j=0;j<8;j++) {
    		flash_read_page(0,i*16+j*2);
			printf("%c%c",flash_page_data,flash_page_data2);
		}
	}
	flash_read_page(0,0x40);
	printf("%c%c",flash_page_data,flash_page_data2);
  	flash_read_page(0,0x42);
	printf("%c%c",flash_page_data,flash_page_data2);
    solar_load_parameter_from_flash();
	printf("\r\n Min Stroke=%lu/256 cm",act_min_stroke);
	printf("\r\n Max Stroke=%lu/256 cm",act_max_stroke);
	printf("\r\n Safty Stroke=%lu/256 cm",act_safety_stroke);
	printf("\r\n Latitude=%lu/100",latitude);
	printf("\r\n Longitude=%lu/100",longitude);
	printf("\r\n Altitude =%lu/100",altitude);
	enable_interrupts(GLOBAL);

}
/*
void display_LED() {
	portd.data_bus=LED_status;
	delay_cycles(20);
	portd.LED_latch=1;
	delay_cycles(20);
	portd.LED_latch=0;
	delay_cycles(20);
	portd.LED_latch=1;
}
*/
////////////////////////////////////////////////////////////////////
void solar_get_act_length(unsigned int8 nActuator) {
	unsigned int16 sun_rise_period;
	unsigned int16 current_period;
	unsigned int16 current_period_fraction;

	unsigned int16 current_act_len;
	unsigned int16 next_act_len;
	
    target_act_position =0;

	flash_read_page(nDay,0x00);  // verify page number
    if (nDay != make16(flash_page_data2,flash_page_data))  {
		printf("Error:Flash data corrupt");
		return;
	}
    current_period = (timer_sec+225)/450;
    current_period_fraction = (timer_sec+225)%450;
    flash_read_page(nDay,0x02); // get sun rise time
    sun_rise_period=make16(flash_page_data2,flash_page_data);
	if ((current_period < sun_rise_period) || (current_period > sun_rise_period+124)) // still dark
	{

		tick = (int32)act_full_stroke_tick[nActuator]*(int32)(act_safety_stroke-act_min_stroke);
		tick = tick/(act_max_stroke-act_min_stroke);
		target_act_position = (unsigned int16) tick;
	} else {
		flash_read_page(nDay,(int8)(current_period-sun_rise_period)*2+4);
		current_act_len = make16(flash_page_data2,flash_page_data);
		flash_read_page(nDay,(int8)(current_period-sun_rise_period)*2+6);
		next_act_len = make16(flash_page_data2,flash_page_data);
		// do linear interpolation of the act len
		if (next_act_len>current_act_len) {
			tick = (int32)(next_act_len-current_act_len)*(int32)(current_period_fraction);
			tick = tick/450;
        	next_act_len = (unsigned int16) tick;
			current_act_len = current_act_len + next_act_len;
		} else  {
			tick = (int32)(current_act_len-next_act_len)*(int32)(current_period_fraction);
			tick = tick/450;
        	next_act_len = (unsigned int16) tick;
			current_act_len = current_act_len - next_act_len;
		}
		if (current_act_len >= act_max_stroke) current_act_len = act_max_stroke;
		if (current_act_len <= act_min_stroke) current_act_len = act_min_stroke;

		tick = (int32)act_full_stroke_tick[nActuator]* (int32)(current_act_len-act_min_stroke);
		tick = tick/(act_max_stroke-act_min_stroke);
		target_act_position = (unsigned int16) tick;
	}

}

////////////////////////////////////////////////////////////////////
void move_act(int16 nPulse,int16 time_out_sec,int16 stuck_sec,int8 direction,unsigned int8 nActuator) {
   // direction 0 = east, 1=west
   unsigned int16 xxx=0;
   unsigned int16 yyy=0;   
   output_low(CCW);
   output_low(EN0);
   output_low(EN1);
   output_low(EN2);
   output_low(EN3);
   delay_ms(20);
   if (nActuator > 3) return; // actuator exceed max number allowable
   flag2.is_moving =1;
   switch (nActuator) {
		case 0: { flag.prev_pulse_state = input(SENSE_0); break;}
		case 1: { flag.prev_pulse_state = input(SENSE_1); break;}
		case 2: { flag.prev_pulse_state = input(SENSE_2); break;}
		case 3: { flag.prev_pulse_state = input(SENSE_3); break;}
   }

   if (!direction)   {
      output_high(CCW);
   }

   delay_ms(200);
   switch (nActuator) {
		case 0: {output_high(EN0); break;}
		case 1: {output_high(EN1); break;}
		case 2: {output_high(EN2); break;}
		case 3: {output_high(EN3); break;}
   }
   delay_ms(10);

   tick = timer_sec;
   tick2 = timer_sec;
   actuator_pulse =0;
   while(1) {
      restart_wdt();
      xxx = (int16) (timer_sec-tick);
      yyy = (int16) (timer_sec-tick2);
   	  switch (nActuator) {
		case 0: { flag2.current_pulse_state = input(SENSE_0); break;}
		case 1: { flag2.current_pulse_state = input(SENSE_1); break;}
		case 2: { flag2.current_pulse_state = input(SENSE_2); break;}
		case 3: { flag2.current_pulse_state = input(SENSE_3); break;}
      }

	  if (flag2.allow_manual_move_act) {
		  if (((!direction && (nButton==3)) || (direction && (nButton==5))) && flag2.button_pressed) 
			flag2.allow_manual_move_act=1;
		  else
			flag2.allow_manual_move_act=0;	
	  }
	  if (!flag2.allow_manual_move_act) { 
      if (flag2.abort_current_activity ==1) {
		 flag2.abort_current_activity=0;
		 break;
      }
      if ( xxx>= time_out_sec) 
         break;
       if (actuator_pulse >= nPulse)
	    switch (nActuator) {
			case 0: {output_low(EN0); break;}
			case 1: {output_low(EN1); break;}
			case 2: {output_low(EN2); break;}
			case 3: {output_low(EN3); break;}
   		}
      if ( yyy >=stuck_sec)
         break;
      }   
      if (flag.prev_pulse_state != flag2.current_pulse_state) {
         tick2 = timer_sec;
         flag.prev_pulse_state = flag2.current_pulse_state;
         if (!direction && (current_act_position[nActuator] >0)) current_act_position[nActuator]--;
         if (direction && (current_act_position[nActuator] < act_full_stroke_tick[nActuator])) current_act_position[nActuator]++;
         actuator_pulse=actuator_pulse+1;
	     printf("\r%ld",actuator_pulse);
		 //led_status.operation = !led_status.operation;
		 //display_LED();
      }
   }
   flag2.abort_current_activity=0;
   output_low(EN0);
   output_low(EN1);
   output_low(EN2);
   output_low(EN3);
   delay_ms(500);
   output_low(CCW);
   delay_ms(20);
   //led_status.operation = 1;
   //display_LED();
   flag2.is_moving =0;


}

void actuator_move_execute(nActuator) {
    if(current_act_position[nActuator] > (target_act_position+5))// move east
            move_act(current_act_position[nActuator]-target_act_position,4500,move_act_time_out,0,nActuator);
    else if((current_act_position[nActuator]+5) < target_act_position)// move west
            move_act(target_act_position-current_act_position[nActuator],4500,move_act_time_out,1,nActuator);
    else if (target_act_position == 0x00)  // move east all the way
            move_act(act_full_stroke_tick[nActuator],4500,move_act_time_out,0,nActuator);
    else if (target_act_position == act_full_stroke_tick[nActuator]) // move west all the way
            move_act(act_full_stroke_tick[nActuator],4500,move_act_time_out,1,nActuator);
}



///////////////////////////////////////////////////////////
void read_eeprom_data()
{
   int8 i,j;
   int16 temp_mem;   

   temp_mem= &timer_sec;
   for (i=0;i<4;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_TIME+i),1);

   temp_mem= &nDay;
   for (i=0;i<4;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_TIME+i+4),1);


   temp_mem= &startup_counter;
   for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_START_COUNTER+i),1);

   for (j=0;j<4;j++) {
   		temp_mem= &act_full_stroke_tick[j];
	   	for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_FULL_STROKE+i+j*2),1);
   		if (act_full_stroke_tick[j] > MAX_FULL_STROKE) act_full_stroke_tick[j] = MAX_FULL_STROKE;
   		temp_mem= &current_act_position[j];
	    for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_CURRENT_STROKE+i+j*2),1);
   		if (current_act_position[j] > act_full_stroke_tick[j]) current_act_position[j] = act_full_stroke_tick[j];
   }


}

void write_eeprom_data(int8 write_cal)
{
   int8 i,j;
   for (i=0;i<4;i++) write_eeprom((int8)ADDR_TIME+i,timer_sec>>(i*8));
   for (i=0;i<4;i++) write_eeprom((int8)ADDR_TIME+i+4,nDay>>(i*8));

   for (j=0;j<4;j++) for (i=0;i<2;i++) write_eeprom((int8)ADDR_CURRENT_STROKE+i+j*2,current_act_position[j]>>(i*8));
   for (i=0;i<2;i++) write_eeprom((int8)ADDR_START_COUNTER+i,startup_counter>>(i*8));
   if (write_cal ==1)
      for (j=0;j<4;j++) for (i=0;i<2;i++) write_eeprom((int8)ADDR_FULL_STROKE+i+j*2,act_full_stroke_tick[j]>>(i*8));
}




////////////////////////////////////////////////////////
