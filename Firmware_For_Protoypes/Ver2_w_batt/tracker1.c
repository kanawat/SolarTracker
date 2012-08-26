// 22-may-2010 always enable high voltage power supply (ps_en=1)

#include <18F458.h>
#device ADC=10
#include <stdlib.h>
#include <math.h>
#fuses HS,NOWDT,WDT128,NOPROTECT,NOLVP,PUT,NOBROWNOUT
#use fixed_io(d_outputs=PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7)
#use delay(clock=10000000)
#use rs232(baud=57600, xmit=PIN_C6, rcv=PIN_C7, BRGH1OK, parity =O)





struct flag {
   boolean task1_armed;
   boolean update_time;
   boolean prev_pulse_state;
   boolean cmd_posted;
   boolean setup_required;
   boolean reset_rs232;
   boolean pwr_state;
   boolean measured_current;
} flag;

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
#byte PORTA =0xF80
#byte PIE1 = 0xF9D
#byte RCSTA= 0xFAB
#byte TXSTA= 0xFAC
#byte PIR1 = 0xF9E
#byte TMR1H = 0xFCF
#byte T1CON = 0xFCD
////////////// various constant /////////
#define Y2010_UNIX_TIME 1262304000
#define Y2010_JDN 2455197.5
#define UTC 7
#define SEC_IN_4_YEARS 126230400
#define MAX_CMD_LEN 18
#define TX_DLY_TIME 1
////////////// actuator parameter /////////
#define DIM_A 107.5
#define DIM_P 9.3
#define DIM_M 5.5
#define DIM_K 36.0
#define DIM_B 15.8
#define MIN_STROKE 70.3 
#define MAX_STROKE 128.0
#define ROW_SPACING 320.00
#define PANEL_WIDTH 160.00
/////////////////  I/O ////////////
#define EN0 PIN_B0
#define EN1   PIN_B1
#define EN2 PIN_B2
#define EN3   PIN_B3
#define CCW PIN_B4
#define FLASH_SELECT PIN_C2 // output
#define FLASH_CLOCK  PIN_C3 // output
#define FLASH_DI     PIN_C5 // output
#define FLASH_DO     PIN_C4 // input
#define SENSE_0       PIN_A1
#define SENSE_1       PIN_A2
#define SENSE_2       PIN_A3
#define SENSE_3       PIN_A4
#define WALL_PWR    PIN_A5 //input
///////////// DATA STORAGE ////////
#define ADDR_FULL_STROKE    0xf00000
#define ADDR_CURRENT_STROKE 0xf00002
#define ADDR_TIME           0xf00004
#define ADDR_START_COUNTER  0xf00008
#define MAX_FULL_STROKE 2000
#rom ADDR_FULL_STROKE={0x05B1} // default value = 0x2D8 = 728 tick
#rom ADDR_CURRENT_STROKE={0x0000}
#rom ADDR_TIME={0x0000,0x0000}
#rom ADDR_START_COUNTER={0x0000}
//#rom ADDR_TIME={0x1C20,0x0000}

const int8 line[4] = {0,0x40,0x10,0x50};
unsigned int32 timer_sec =0;
unsigned int32 tick =0;
unsigned int32 tick2 =0;
char tmp_str[20],tmp_str2[20];
char cmd_msg[20];
int8 cmd_len=0;
unsigned int8 timer_ms=0;
unsigned int32 next_sun_rise;
unsigned int16 current_act_position=0;
unsigned int16 target_act_position=0;
unsigned int32 JDN,jj,g;
unsigned int32 dg,c,dc,b,db,a,da,y,m,d;
unsigned int32 YY,MM,DD,time_of_day;
unsigned int32 sec_until_sun_rise;
int16 FULL_STROKE_TICK=0;
float al;
int16 move_act_time_out=3;
int8 tx_delay=0;
int16 current_measured=0;
int8 n_avg_current_measured=0;
int16 startup_counter=0;
int16 actuator_pulse=0;
unsigned int8 index_in_page;
float len_a,len_b,len_c,len_p,len_m,len_k,len_l;
float sin_beta,cos_beta,tan_beta;
float temp1;
float alpha;
float act_len;


void init_ext_flash(void);
void ext_flash_startContinuousRead(int pageAddress);
void ext_flash_sendData(int data, int size);
void ext_flash_send16Data(int16 data, int size);
void ext_flash_sendBytes(char* data, int size);
void ext_flash_getBytes(char* data, int16 size);
void ext_flash_readPage(int16 pageAddress, int pageIndex, char* data, int16 size);
void ext_flash_writePageThroughBuffer(int pageAddress, char* data, int size);
int ext_flash_getByte(void);
void ext_flash_waitUntilReady(void);

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


void read_eeprom_data()
{
   int i;
   int16 temp_mem;   
   temp_mem= &timer_sec;
   for (i=0;i<4;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_TIME+i),1);
   temp_mem= &FULL_STROKE_TICK;
   if (FULL_STROKE_TICK > MAX_FULL_STROKE) FULL_STROKE_TICK = MAX_FULL_STROKE;
   if (FULL_STROKE_TICK <0) FULL_STROKE_TICK = 0;

   for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_FULL_STROKE+i),1);
   temp_mem= &current_act_position;
   if (current_act_position > FULL_STROKE_TICK) current_act_position = FULL_STROKE_TICK;
   if (current_act_position < 0) current_act_position =0;

   for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_CURRENT_STROKE+i),1);
   temp_mem= &startup_counter;
   for (i=0;i<2;i++) memset(temp_mem+i,read_eeprom((int8)ADDR_START_COUNTER+i),1);
}

void write_eeprom_data(int8 write_cal)
{
   int8 i;
   for (i=0;i<4;i++) write_eeprom((int8)ADDR_TIME+i,timer_sec>>(i*8));
   for (i=0;i<2;i++) write_eeprom((int8)ADDR_CURRENT_STROKE+i,current_act_position>>(i*8));
   for (i=0;i<2;i++) write_eeprom((int8)ADDR_START_COUNTER+i,startup_counter>>(i*8));
   if (write_cal ==1)
      for (i=0;i<2;i++) write_eeprom((int8)ADDR_FULL_STROKE+i,FULL_STROKE_TICK>>(i*8));
}


void init_ext_flash(void) {
  output_low(FLASH_CLOCK);
  output_high(FLASH_SELECT);
}

/*
void ext_flash_startContinuousRead(int pageAddress) {
  ext_flash_waitUntilReady();
  output_low(FLASH_SELECT);
  ext_flash_sendData(0xE8, 8);
  ext_flash_sendData(pageAddress, 14);
  ext_flash_sendData(0, 10);
  ext_flash_sendData(0, 16);
  ext_flash_sendData(0, 16);
}

*/

int ext_flash_getByte(void) {
  int flashData = 0;
  output_high(FLASH_CLOCK); flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
  return(flashData);
}

void ext_flash_getBytes(char* data, int16 size) {
   int16 i, j;
   for(i = 0; i < size; i++) {
      for(j = 0; j < 8; j++) {
         output_high(FLASH_CLOCK);
         shift_left(data + i, 1, input(FLASH_DO));
         output_low(FLASH_CLOCK);
      }
   }
}

void ext_flash_readPage(int16 pageAddress, int pageIndex, char* data, int16 size) {
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0xD2, 8);
   ext_flash_send16Data(pageAddress, 15);
   ext_flash_sendData(0, 1);
   ext_flash_sendData(pageIndex, 8);
   ext_flash_sendData(0, 16);
   ext_flash_sendData(0, 16);
   ext_flash_getBytes(data, size);
   output_high(FLASH_SELECT);
}

/*
void ext_flash_writePageThroughBuffer(int pageAddress, char* data, int size) {
   int i;
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0x82, 8);
   ext_flash_sendData(pageAddress, 14);
   ext_flash_sendData(0, 10);
   for (i = 0; i < size; i++)
     ext_flash_sendData((int)data[i], 8);
   output_high(FLASH_SELECT);
}
*/


void ext_flash_sendData(int data, int size) {
   do {
      size--;
      output_bit(FLASH_DI, (data >> size) & 1);
      output_high(FLASH_CLOCK);
      output_low(FLASH_CLOCK);
   } while(size > 0);
}

void ext_flash_send16Data(int16 data, int size) {
   do {
      size--;
      output_bit(FLASH_DI, (data >> size) & 1);
      output_high(FLASH_CLOCK);
      output_low(FLASH_CLOCK);
   } while(size > 0);
}

void ext_flash_sendBytes(char* data, int size) {
   int i;
   for (i = 0; i < size; i++)
     ext_flash_sendData((int)data[i], 8);
}

void ext_flash_waitUntilReady(void) {
// this function read status register
// Bit7 : 1=RDY, 0=BUSY
// Bit6 : COMP, 1=Recent Main memory page compare, 0 == match
// Bit 5-2 : 0b0111 , density code for 4MBit
// Bit 1: Protect
// Bit 0: Page size. 0= 264 byte, 1=256 byte

  int flashData;
  int i;
  output_low(FLASH_SELECT);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 0); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 0); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  output_bit(FLASH_DI, 1); output_high(FLASH_CLOCK); output_low(FLASH_CLOCK);
  for(i=0;i<255;i++) {
    flashData = 0;
    output_high(FLASH_CLOCK); flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    output_high(FLASH_CLOCK); flashData *= 2; flashData += input(FLASH_DO); output_low(FLASH_CLOCK);
    if (bit_test(flashData,7) == 1) break; // device is not busy
   delay_cycles(50);
   restart_wdt();
  }
  output_high(FLASH_SELECT);
}

void ext_flash_block_erase() {
   int i;
   for(i=0;i<0xFF;i++) {
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0x50, 8);
   ext_flash_sendData(0, 4);
   ext_flash_sendData(i, 8);
   ext_flash_sendData(0, 12);
   output_high(FLASH_SELECT);
   }
   ext_flash_waitUntilReady();
}


void ext_flash_buffer1_read() {
   char data;
   int i;
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0xD1, 8);
   ext_flash_send16Data(0, 16);
   ext_flash_sendData(0, 8);
   i=255;
   do {
     ext_flash_getBytes(&data,1);
     i++;
     if (i%16==0) printf("\r\n %02X:", i);
      printf("%02X ",data);

   } while (i<255);
   output_high(FLASH_SELECT);   
}


void ext_flash_buffer1_write(int8 data, int8 PageIndex, int8 nData) {
   int i;
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0x84, 8);
   ext_flash_send16Data(0, 16);
   ext_flash_sendData(PageIndex, 8);
   if (nData>1) {
   i=255;
   do {
      ext_flash_sendData(data,8);
      i++;
   } while(i<nData);
   } else
         ext_flash_sendData(data,8);
   output_high(FLASH_SELECT);
}

void ext_flash_write_buffer1_to_main_memory(int16 pageAddress) {
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0x83, 8);
   ext_flash_send16Data(pageAddress, 15);
   ext_flash_send16Data(0, 9);
   output_high(FLASH_SELECT);
}

void ext_flash_main_memory_to_buffer1(int16 pageAddress) {
   ext_flash_waitUntilReady();
   output_low(FLASH_SELECT);
   ext_flash_sendData(0x53, 8);
   ext_flash_send16Data(pageAddress, 15);
   ext_flash_send16Data(0, 9);
   output_high(FLASH_SELECT);
}



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
   

void print_date_time() {

      JDN = (unsigned int32)((float) (Y2010_JDN*1.0+ 0.5+ (float)(UTC*1.0)/24.0 + (timer_sec)*1.0/86400.0));
      jj = JDN +32044;
      g=jj/146097;
      dg=jj%146097;
      c=(dg/36524+1)*3/4;
      dc=dg-c*36524;
      b=dc/1461;
      db=dc%1461;
      a=(db/365+1)*3/4;
      da=db-a*365;
      y=g*400+c*100+b*4+a;
      m=(da*5+308)/153-2;
      d=da-(m+4)*153/5+122;
      YY=y-4800+(m+2)/12;
      MM=(m+2)%12+1;
      DD=d+1;
      time_of_day = (timer_sec+((int8)UTC)*3600)%86400;

      strcpy(tmp_str,"00/00/00");
      itoa((int8)DD,10,tmp_str2);
      if (strlen(tmp_str2)==1) tmp_str[1]=tmp_str2[0]; else memcpy(tmp_str,tmp_str2,2);
      itoa((int8)MM,10,tmp_str2);
      if (strlen(tmp_str2)==1) tmp_str[4]=tmp_str2[0]; else memcpy(tmp_str+3,tmp_str2,2);
      itoa((int8)(YY%2000),10,tmp_str2);
      if (strlen(tmp_str2)==1) tmp_str[7]=tmp_str2[0]; else memcpy(tmp_str+6,tmp_str2,2);
      lcd_gotoxy(0,0);
      lcd_put_str(tmp_str,strlen(tmp_str));
      strcpy(tmp_str,"00:00:00");
      itoa((int8) (time_of_day/3600),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+2-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      itoa((int8) ((time_of_day%3600)/60),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+5-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      itoa((int8) (time_of_day%60),10,tmp_str2);
      if (strlen(tmp_str2)<=2) memcpy(tmp_str+8-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      lcd_gotoxy(0,1);
      lcd_put_str(tmp_str,strlen(tmp_str));

      strcpy(tmp_str,"   ");
      itoa((int8)startup_counter,10,tmp_str2);
      if (strlen(tmp_str2)<=3) memcpy(tmp_str+3-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
       lcd_gotoxy(0,2);
      lcd_put_str(tmp_str,strlen(tmp_str));         


}

void download_sun_table(int16 nPage) {
    int16 i,j;
   disable_interrupts(GLOBAL);
   restart_wdt();
   setup_wdt(WDT_OFF);
    output_high(FLASH_SELECT);
    while(kbhit()) getc();
    for (j=0;j<nPage+1;j++) {
        ext_flash_waitUntilReady();
      PORTE.tx_en=1;
        printf("OK\r\n");
      tx_delay=TX_DLY_TIME;
      //PORTE.tx_en=0;
         output_low(FLASH_SELECT);
        if (j%2) ext_flash_sendData(0x82, 8); else ext_flash_sendData(0x85, 8);
         ext_flash_send16Data(j+10, 15); // page address
      ext_flash_send16Data(0, 9);
      for (i=0;i<256;i++) {
            ext_flash_sendData(getc(), 8);
      }
       output_high(FLASH_SELECT);
   }
   enable_interrupts(GLOBAL);
   restart_wdt();
   setup_wdt(WDT_ON);

}

/*
void print_page_data(int16 nPage) {
    char temp[256];
    int16 i;
    int32 time_index;

   ext_flash_readPage(nPage,0,temp,264);
    for(i=0;i<264;i++) {
      PORTE.tx_en=1;
      if((i%16) ==0) printf("\r\n%02X : ",i);
      printf("%02X ",temp[i]);
   }
   memcpy(&time_index,temp,4);
    printf("\r\n Timer Index: %Lu",time_index);   
   


}
*/
void print_page_data(int16 nPage) {
    char temp[17];
    int8 i;
   int8 j;
    PORTE.tx_en=1;
    for(i=0;i<16;i++) {
      printf("\r\n%02X : ",i);
      ext_flash_readPage(nPage,i*16,temp,16);
      for (j=0;j<16;j++) printf("%02X ",temp[j]);
   }
   tx_delay=TX_DLY_TIME;
}

int32 get_timer_index(int16 nDay) { // 1 page contain 2 subpage
    char temp[6];
    int32 time_index;
   ext_flash_readPage((nDay/2)+10,128*(nDay%2),temp,4);
   memcpy(&time_index,temp,4);
   return time_index;
}

int16 find_day_number(int32 tsec) // return day number after 1-jan-2010, we use this to search for sun angle in flash memory page
{
   return (((tsec+((int8)UTC)*3600)/86400)%1461);
}

unsigned int8 current_sun_angle(unsigned int32 t_sec) {
   unsigned int16 today;  // current day (count after epoch)
   unsigned int32 sun_rise_t_sec; // time of day . compensated for time zone
   unsigned int8 sun_data;
   today = find_day_number(t_sec); // day after epoch
    sun_rise_t_sec = get_timer_index(today)*450;
    if (sun_rise_t_sec > t_sec%SEC_IN_4_YEARS) return 0x0; // sun havent up yet, angle is 0
   index_in_page = (t_sec%SEC_IN_4_YEARS-sun_rise_t_sec)/450;
    if (index_in_page>=124) return 0xFF; // exceed page size , sun went down
    ext_flash_readPage((today/2)+10,128*(today%2)+4+index_in_page,&sun_data,1);
   if ((index_in_page>5) && (sun_data==0)) return 0xFF; // sun went down
   return sun_data;
   
}


void move_act(int16 nPulse,int16 time_out_sec,int16 stuck_sec,boolean direction,boolean abort_when_task_armed) {
   // direction 0 = east, 1=west
   unsigned int16 xxx=0;
   unsigned int16 yyy=0;   
   int8 rt=5;
   output_low(CCW);
   output_low(EN0);
   output_low(EN1);
   output_low(EN2);
   output_low(EN3);
   PORTD.ps_en=1;
    delay_ms(20);

   if (!direction)   {
      output_high(CCW);
   }

   PORTD.ps_en=1;
    delay_ms(20);
   output_high(EN0);
    delay_ms(10);

   flag.prev_pulse_state = input(PIN_A2); //A2 = sensor wire for actuator 1
   tick = timer_sec;
   tick2 = timer_sec;
   actuator_pulse =0;
   while(1) {
      xxx = (int16) (timer_sec-tick);
      yyy = (int16) (timer_sec-tick2);
      if ( xxx>= time_out_sec) 
         break;
       if (actuator_pulse >= nPulse)
         output_low(EN0);
      if ( yyy >=stuck_sec)
         break;
      if (abort_when_task_armed && (flag.task1_armed||flag.cmd_posted))
         break;
      if (flag.prev_pulse_state != input(SENSE_0)) {
         tick2 = timer_sec;
         flag.prev_pulse_state = input(SENSE_0);
         if (!direction && (current_act_position >0)) current_act_position--;
         if (direction && (current_act_position < FULL_STROKE_TICK)) current_act_position++;
         actuator_pulse=actuator_pulse+1;
         lcd_gotoxy(11,2);
         if (actuator_pulse%2==0)   lcd_putc('*'); else  lcd_putc('.');
         lcd_gotoxy(12,2);
         printf(lcd_putc,"%4lu",current_act_position);
         PORTE.tx_en=1;
         //printf("\r%4lu",actuator_pulse);
         tx_delay=TX_DLY_TIME;

      }
      if (flag.update_time) {
         flag.update_time = false;
         print_date_time();
      }
   }
   lcd_gotoxy(12,2);
    printf(lcd_putc,"%4lu",current_act_position);
   output_low(EN0);
   output_low(EN1);
   output_low(EN2);
   output_low(EN3);
   PORTD.ps_en=1;
    delay_ms(50);
   output_low(CCW);
    delay_ms(50);
}

int8 task1() {
      int8 sun_angle;
      //lcd_init();
      if (timer_sec%SEC_IN_4_YEARS >= next_sun_rise) {
          next_sun_rise=get_timer_index(find_day_number((next_sun_rise+86400)%SEC_IN_4_YEARS))*450;
      }
        sec_until_sun_rise = (int32) (next_sun_rise-timer_sec);
      sun_angle=current_sun_angle(timer_sec%SEC_IN_4_YEARS);

      // prepare buffer1
      if (sun_angle==0x00) // sun havent' come up yet, clear buffer1
      {
         if(find_day_number(timer_sec%SEC_IN_4_YEARS)%2==0) 
            ext_flash_buffer1_write(0xAA,0,255);
         else
            ext_flash_buffer1_write(0xAA,128,127);
      }

      if ((sun_angle == 0xFF) || (sun_angle == 0x00)) // sundown
         sun_angle = 180;

      if (sun_angle != 180) {
         ext_flash_buffer1_write((int8)(current_measured/n_avg_current_measured),128*(find_day_number(timer_sec%SEC_IN_4_YEARS)%2)+4+index_in_page,1);
         ext_flash_write_buffer1_to_main_memory(1010+(find_day_number(timer_sec%SEC_IN_4_YEARS)/2));
      }
      strcpy(tmp_str,"#     ");
      itoa((int16)find_day_number(timer_sec%SEC_IN_4_YEARS),10,tmp_str2);
      memcpy(tmp_str+4-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
      lcd_gotoxy(10,0);
      lcd_gotoxy(10,0);      
      lcd_gotoxy(10,0);
      lcd_put_str(tmp_str,strlen(tmp_str));

      strcpy(tmp_str,"   ");
      tmp_str[3]=0xDF;
      itoa((int8)sun_angle,10,tmp_str2);
      if (strlen(tmp_str2)<=3) memcpy(tmp_str+3-strlen(tmp_str2),tmp_str2,strlen(tmp_str2));
       lcd_gotoxy(10,1);
      lcd_put_str(tmp_str,strlen(tmp_str));         

      return sun_angle;

}

unsigned int8 get_backtrack_angle(unsigned int8 alpha) {

   float tan_alpha,ll,dd;
   float beta;
   ll = PANEL_WIDTH;
   dd = ROW_SPACING;
   tan_alpha = tan( (float) (alpha) *PI/180.0);

   beta = acos(-1*dd/(ll*sqrt(1.0+tan_alpha*tan_alpha))) -atan(-1.0*tan_alpha);
   return fmod(beta,PI)*180.0/PI;
}


#int_timer1
void timer1_ovf()
{   // overflow every 1 sec
      set_timer1(get_timer1()+0x8000);
      timer_sec+=1; 
      flag.update_time = true;
      restart_wdt();
      if ((timer_sec%15) ==0)
         flag.measured_current=true;
      if ((timer_sec%450) ==0)
         flag.task1_armed=true;
      if (timer_sec%300==30) flag.reset_rs232=true;
      if (tx_delay>0) tx_delay--;
//      
}


#int_rda
void recive_cmd()
{
   char c;
    c=getc();
    PORTE.tx_en=1;
   delay_ms(2);
     if(c==8) {  // Backspace
        if(cmd_len>0) {
          cmd_len--;
          printf("%c",c);
          printf(" ");
          printf("%c",c);
        }
     } else if ((c>=' ')&&(c<='~'))
       if(cmd_len<=MAX_CMD_LEN) {
         cmd_msg[cmd_len++]=c;
         printf("%c",c);
       } 
    if (c==13) { 
   printf("\r\n>");
    //PORTE.tx_en=0;
   cmd_msg[cmd_len]=0;
   if(cmd_len>1)flag.cmd_posted=true;
   cmd_len =0;
  }
   tx_delay=TX_DLY_TIME;

}

void actuator_length(int8 sun_angle) {
// return actuator length in cm at given sun_angle
//if (sun_angle < 30) return;
//if (sun_angle > 150) return;
len_a = DIM_A;
len_b = DIM_B;
len_p = DIM_P;
len_k = DIM_K;
len_m = DIM_M;
len_p = DIM_P;
alpha = atan(len_b/len_k);
sin_beta = sin((sun_angle*PI/180.0)-alpha);
cos_beta = cos((sun_angle*PI/180.0)-alpha);
tan_beta = tan((sun_angle*PI/180.0)-alpha);


len_c = sqrt(len_k*len_k+len_b*len_b)-len_p/sin_beta;
temp1 = len_a - (len_p/tan_beta)-len_c*cos_beta;
len_l = len_c*sin_beta*len_c*sin_beta+temp1*temp1-len_m*len_m;
len_l = sqrt(len_l);

if (len_l<50.0) {
   act_len=0.0;
   return;
}

act_len= len_l;
}

void process_cmd_msg(){
    flag.cmd_posted =false;
   switch (cmd_msg[0]) {
      case 'm':{ 
         memcpy(cmd_msg,cmd_msg+1,18);
           if (atol(cmd_msg)==0) break;
         move_act(atol(cmd_msg),9000,move_act_time_out,1,0);
         break;    }

      case 'n':{ 
         memcpy(cmd_msg,cmd_msg+1,18);
           if (atol(cmd_msg)==0) break;
         move_act(atol(cmd_msg),9000,move_act_time_out,0,0);
         break;    }
/*
      case 'x':{
         memcpy(cmd_msg,cmd_msg+1,18);
           if (atol(cmd_msg)==1) {
            setup_timer_0(RTCC_DIV_32);
            move_act_time_out =4;
         } else if(atol(cmd_msg)==2) {
            setup_timer_0(RTCC_DIV_1);
            move_act_time_out =128;
         }
         break;    }   
*/   
      case 'd':{ 
         memcpy(cmd_msg,cmd_msg+1,18);
           if (atol(cmd_msg)==0) break;
         PORTE.tx_en=1;
         printf("\r\n DL %ld pages ..",atol(cmd_msg));
         tx_delay=TX_DLY_TIME;
           download_sun_table(atol(cmd_msg));
         break;    }
      case 'r': {
         disable_interrupts(GLOBAL);
         restart_wdt();
         setup_wdt(WDT_OFF);
         memcpy(cmd_msg,cmd_msg+1,18);
         PORTE.tx_en=1;
         printf("\r\n PAGE %ld:",atol(cmd_msg));
         tx_delay=TX_DLY_TIME;
         print_page_data(atol(cmd_msg));
         enable_interrupts(GLOBAL);
         restart_wdt();
         setup_wdt(WDT_ON);
            break; }
        case 'e': {
         memcpy(cmd_msg,cmd_msg+1,18);
            if (atol(cmd_msg)!=22) break;
         PORTE.tx_en=1;
         printf("\r\ndeleting entire flash data ");
         tx_delay=TX_DLY_TIME;
         ext_flash_block_erase();
            break; }
      case 't': { // timer set
         memcpy(cmd_msg,cmd_msg+1,18);
         timer_sec=atoi32(cmd_msg);
         lcd_send_cmd(0,0x02);
         next_sun_rise=get_timer_index(find_day_number((timer_sec)%SEC_IN_4_YEARS))*450;
         enable_interrupts(INT_TIMER1);   
         enable_interrupts(GLOBAL); 
         flag.task1_armed=true;
         write_eeprom_data(0); // save full_stroke_tick and current_position
         flag.setup_required =false;
         break;
      }
      case 'h' : { //return home
         move_act(2000,9000,move_act_time_out,1,0); // move actuator to west
         move_act(2000,9000,move_act_time_out,0,0); // move actuator to home position
         FULL_STROKE_TICK= actuator_pulse;
         current_act_position =0;
         write_eeprom_data(1); // save full_stroke_tick and current_position
         break;
      }
/*
      case 'g': {
         print_date_time();
          break;
      }
      case 'a': {
         memcpy(cmd_msg,cmd_msg+1,18);
         PORTE.tx_en=1;
         printf("\r\n   Sun angle %u, stroke length =%f",atoi(cmd_msg),actuator_length(atoi(cmd_msg)));
         tx_delay=TX_DLY_TIME;
         break;
      }
*/
      case 's': {
         memcpy(cmd_msg,cmd_msg+1,18);
            if (atol(cmd_msg)!=22) break;
         write_eeprom_data(1);
         break;
      }
      case 'y': { //adc conversion
         set_adc_channel( 0 );
         delay_us(20);
         current_measured = (read_adc());
         printf("\r\nADC= %lu",current_measured);
         break;
      }
      case 'w': { //buffer1 read
         //ext_flash_buffer1_write(0x00,0x00, 1);
         //ext_flash_buffer1_read();
         //ext_flash_buffer1_write(0x12,0xAA, 0);
         disable_interrupts(GLOBAL);
         restart_wdt();
         setup_wdt(WDT_OFF);
         ext_flash_buffer1_read();
         enable_interrupts(GLOBAL);
         restart_wdt();
         setup_wdt(WDT_ON);

         //ext_flash_write_buffer1_to_main_memory(1024);
         break;
      }
      case 'i': { //adc conversion
         printf("\r\nAct:%lu/%lu",current_act_position,FULL_STROKE_TICK);
         printf("\r\nRestart:%lu",startup_counter);
         printf("\r\nCurrent:%lu (%d)",current_measured/n_avg_current_measured, n_avg_current_measured);
         printf("\r\nTimer:%lu",timer_sec);
         printf("\r\n#Day:%lu",find_day_number((timer_sec)%SEC_IN_4_YEARS));

         break;
      }
      case 'c': { //clear write buffer
          ext_flash_buffer1_write(0xAA,0,255);
         break;
      }

      case 'x': { //stop timer
         flag.setup_required=true;         
         restart_wdt();
         setup_wdt(WDT_OFF);
         break;
      }

      case 'o': { //start timer
         flag.setup_required=false;         
         setup_wdt(WDT_ON);
         restart_wdt();
         break;
      }


      }
      
   }

void print_len_and_tick()
{
   lcd_gotoxy(0,3);
   itoa((int16)(al),10,tmp_str);
    printf(lcd_putc,"L=%s.",tmp_str);
   itoa((int16)(al*100.0)%100,10,tmp_str);
    printf(lcd_putc,"%scm",tmp_str);
   lcd_gotoxy(11,3);
    printf(lcd_putc,"*%4lu",target_act_position);

}


void main() {
    unsigned int8 solar_angle;
   int16 tick=0;
   int16 temp_mem;
   int8 i;
    set_tris_d(0x00); // all D are output
   set_tris_e(0b00000010); // RE1 = emergnecy input
   set_tris_a(0b11111111); // A0 -1 is output
   set_tris_b(0xA0);
   set_tris_c(0b10010000);
   timer_sec=0;
   delay_ms(200);
   read_eeprom_data();
   restart_wdt();
   setup_wdt(WDT_OFF);
   setup_adc_ports(AN0);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel( 0 );
   current_measured =0;
   startup_counter++;
   printf("\r\nVer2.1 Restart Cause:%3d",restart_cause());
   write_eeprom_data(0);
   output_low(CCW);
   output_low(EN0);
   output_low(EN1);
   output_low(EN2);
   output_low(EN3);
   PORTD.ps_en=1;
   PORTD.disp_en=0; 
   PORTE.tx_en=1;
   lcd_init();
   init_ext_flash();
   restart_wdt();
   portd.pwr_ctrl=0;
    output_high(FLASH_SELECT);
    setup_timer_0(RTCC_DIV_32);
   T1CON = 0x8F;
   //setup_timer_0(RTCC_DIV_1);
    disable_interrupts(INT_TIMER0);   // Setup interrupt on falling edge
    disable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
    disable_interrupts(GLOBAL);
   flag=0;
    strcpy(cmd_msg,"");
   if (timer_sec ==0x00000000)
      flag.setup_required = true;
   else
      flag.setup_required = false;

/////////////////
   enable_interrupts(INT_RDA);
   enable_interrupts(INT_TIMER1);  
   enable_interrupts(GLOBAL); 
   while(flag.setup_required)   if (flag.cmd_posted) process_cmd_msg();
   setup_wdt(WDT_ON);
   next_sun_rise=get_timer_index(find_day_number((timer_sec)%SEC_IN_4_YEARS))*450;

//////////////////
   // take sun angle and calculate the length of actuator needed to push
   // also need counter of reed switch
   flag.task1_armed =true;
   ext_flash_main_memory_to_buffer1(1010+(find_day_number(timer_sec%SEC_IN_4_YEARS)/2));

   while(1) {

      while(flag.setup_required)   if (flag.cmd_posted) process_cmd_msg();

      if (flag.update_time) {
         flag.update_time = false;
         print_date_time();
      }

      if (flag.measured_current) {   
         flag.measured_current = false;
         current_measured += (read_adc());
         n_avg_current_measured++;
      }

      if (flag.cmd_posted) {
         flag.cmd_posted = false;
         process_cmd_msg();
      }
      if (flag.reset_rs232) {
         flag.reset_rs232 =false;
          init_rs232();
      }
      if (input(WALL_PWR)==false) {
         flag.pwr_state = false;
      } else {
         if (flag.pwr_state==false)
            lcd_init();
         flag.pwr_state=true;
      }
      if (tx_delay==0) PORTE.tx_en=0;
      if (flag.task1_armed) {
         flag.task1_armed = false;
         solar_angle= task1();

         if ((solar_angle>30) && (solar_angle<150)) {
            lcd_gotoxy(0,3);
            actuator_length(solar_angle);
            al=act_len;
         } else {
            if ((solar_angle>0) && (solar_angle<=30)) {
               actuator_length(90-get_backtrack_angle(90-solar_angle));
               al=act_len;
            } else if ((solar_angle>=150) && (solar_angle<=180)) {
               lcd_gotoxy(0,3);
               actuator_length(90+get_backtrack_angle(solar_angle-90));
               al=act_len;
            }
         }
         if((al >= MIN_STROKE) && (al <=MAX_STROKE)) {
            tick = ((al-MIN_STROKE)/(MAX_STROKE-MIN_STROKE))*FULL_STROKE_TICK;
         } else if (al<MIN_STROKE) {
            tick = 0;
         } else if (al>MAX_STROKE) {
            tick = FULL_STROKE_TICK;
         }
         lcd_init();
         target_act_position = tick;
         write_eeprom_data(0);
         print_len_and_tick();

         if(current_act_position > (target_act_position+5))// move east
         {
            move_act(current_act_position-target_act_position,4500,move_act_time_out,0,1);
         } else if((current_act_position+5) < target_act_position)// move west
         {
            move_act(target_act_position-current_act_position,4500,move_act_time_out,1,1);
         } else if (target_act_position == 0x00)  // move east all the way
            move_act(FULL_STROKE_TICK,4500,move_act_time_out,0,1);
         else if (target_act_position == FULL_STROKE_TICK) // move west all the way
            move_act(FULL_STROKE_TICK,4500,move_act_time_out,1,1);
         //delay_ms(20);
         //task1();
         print_len_and_tick();
         write_eeprom_data(0);
         PORTE.tx_en=1;
         current_measured=0;
         n_avg_current_measured=0;
         printf(".");
         tx_delay=TX_DLY_TIME;
         sleep();
      }
   }

}
