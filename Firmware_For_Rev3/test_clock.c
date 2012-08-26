#include <solar_lib.h>
//////////////////////////////////
void button_scan() {
	int8 i;
    flag2.button_pressed=0;
	for(i=0;i<6;i++) {
		portd.data_bus=i;
		portd.MUX_en =0;
		delay_cycles(20);
		if (!input(PIN_E0)) {
			flag2.button_pressed=1;
			nButton=i;
		} 
		delay_cycles(20);
		portd.MUX_en =1;
	}
	restart_wdt();
}


//////////////////////////////////
#int_timer1
void timer1_ovf()
{   // overflow every 1 sec
      //set_timer1(get_timer1()+0x7FB8);
	  set_timer1(0xFFF0);
	  led_status.power = !led_status.power; // blink power led
	  display_LED();

}

void init_spi() {
	setup_spi(spi_master |spi_h_to_l | spi_clk_div_16 );
    SSPSTAT = 0xC0;
}

#int_rda
void recive_cmd()
{
   char c;
   c=getc();
   switch (c) {	
	 case(8): 	cmd_len--; break;
     case(13): 	cmd_msg[cmd_len]=0;
   				if(cmd_len>=1)flag.cmd_posted=true;
   				cmd_len =0;
				break;
     case(10):  break;
     default: 	if ((c>=' ')&&(c<='~')) if(cmd_len<=MAX_CMD_LEN) cmd_msg[cmd_len++]=c;
				break; 
   }
}

void process_cmd_msg(){
   int8 i,j;
   flag.cmd_posted =false;
   printf("\r\n>%s\r\n",cmd_msg);
   switch (cmd_msg[0]) {
      case 'e': {
         memcpy(cmd_msg,cmd_msg+1,18);
         if (atol(cmd_msg)!=22) break;
		 printf("\r\n Block erase\r\n");
         flash_block_erase();
         break; }

      case 'r': {
         disable_interrupts(GLOBAL);
         memcpy(cmd_msg,cmd_msg+1,18);
         printf("\r\n PAGE %ld:",atol(cmd_msg));
         print_page_data(atol(cmd_msg));
         enable_interrupts(GLOBAL);
         break;
      }
      case 'b': {
         disable_interrupts(GLOBAL);
         memcpy(cmd_msg,cmd_msg+1,18);
         printf("\r\n Write buffer1 -> PAGE %ld:",atol(cmd_msg));
         flash_write_buffer1_to_main_memory(atol(cmd_msg));
         enable_interrupts(GLOBAL);
         break;
      }

      case 'c': {
         disable_interrupts(GLOBAL);
         memcpy(cmd_msg,cmd_msg+1,18);
         printf("\r\n Read PAGE %ld -> buffer1",atol(cmd_msg));
         flash_read_main_memory_to_buffer1(atol(cmd_msg));
         enable_interrupts(GLOBAL);
         break;
      }

      case 'f': {
         disable_interrupts(GLOBAL);
		 setup_WDT(WDT_OFF);
         memcpy(cmd_msg,cmd_msg+1,18);
         printf("\r\n WRITE PAGE %ld:",atol(cmd_msg));
         flash_write_page(atol(cmd_msg));
		 printf("\r\n.");
		 setup_WDT(WDT_ON);
         enable_interrupts(GLOBAL);
         break;
      }
      case 'g': {
         print_date_time();
         break;
      }

      case 'w': { //buffer1 read
         disable_interrupts(GLOBAL);
		 printf("\r\n Buffer1:");
    	for(i=0;i<16;i++) {
      		printf("\r\n%02X : ",i);
      		for (j=0;j<8;j++) {
        		flash_buffer1_read(i*16+j*2);
				printf("%02X %02X ",flash_page_data,flash_page_data2);
				restart_wdt();
	  	 	}
         enable_interrupts(GLOBAL);         
         break;	}
      }
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
      case 'p': {
         memcpy(cmd_msg,cmd_msg+1,18);
         if (atol(cmd_msg)!=22) break;
		 printf("\r\n Set 256 page size\r\n");
         flash_set_256_page_size();
         break; }

	  case 'z': {
		 print_fw_info();
		 printf("\r\n startup = %ld",startup_counter);
		 break;
	  }
	  case 'd': {
         memcpy(cmd_msg,cmd_msg+1,18);
         if (atol(cmd_msg)==0) break;
         nDay= atol(cmd_msg);
         break;    }
	  case 't': {
         memcpy(cmd_msg,cmd_msg+1,18);
         timer_sec = atoi32(cmd_msg);
		 timer_sec = timer_sec; // rouding to 60 sec
         break;    }
	  case 'l': {
         memcpy(cmd_msg,cmd_msg+1,18);
         solar_get_act_length(atoi(cmd_msg));
		 disable_interrupts(GLOBAL);
		 printf("\r\n Act#%d Len=%lu Now=%lu\r\n",atoi(cmd_msg), target_act_position, current_act_position[atoi(cmd_msg)]);
		 enable_interrupts(GLOBAL);
		 actuator_move_execute(atoi(cmd_msg));
         break;    }
      case 'h' : { //return home
	     memcpy(cmd_msg,cmd_msg+1,18);
         if(strlen(cmd_msg)>0) {
	     flag2.en_operate =0;
         move_act(2000,9000,move_act_time_out,1,atoi(cmd_msg)); // move actuator to west
         move_act(2000,9000,move_act_time_out,0,atoi(cmd_msg)); // move actuator to home position
         act_full_stroke_tick[atoi(cmd_msg)]= actuator_pulse;
         current_act_position[atoi(cmd_msg)] =0;
         write_eeprom_data(1); // save full_stroke_tick and current_position
	     flag2.en_operate =1; 
		 }

         break;
      }
		 
      case 'j': {
         write_eeprom_data(1); // save full_stroke_tick and current_position
         break;
      }

	  case 'o': {
		 flag2.en_operate = !flag2.en_operate;
		 if (flag2.en_operate) printf("\r\n Normal operation"); else printf("\r\n Halt operation");
	  }

      case 'y': { //adc conversion
         set_adc_channel(0);
         delay_us(20);
		 wall_pwr_read =read_adc();
         printf("\r\nADC= %lu",wall_pwr_read);
         break;
      }
      case 'k': {
		 disable_interrupts(GLOBAL);
         while(1);
         break;
      }



      }// end case      
}
//////////////////////////////////////
void button_menu() {
	int8 k;
	int8 target_act=0;
    flag2.en_operate = 0; // enter halt operation
	LED_status.power = 0; // always red
	LED_status.operation =0;
	LED_status.aux =3; // both turned off
	display_LED();
    flag2.abort_current_activity =0;
	while(flag2.button_pressed ==1)	button_scan();
	delay_ms(10);
	// button release

    while(1) {
		nButton =-1;
		button_scan();
		restart_wdt();
		if (nButton != -1){
		switch (nButton) {
			case 0: { // operate/halt button 
					while (flag2.button_pressed ==1) button_scan();
					delay_ms(10);
					nButton=-1;
					flag2.en_operate =1;
					flag2.abort_current_activity =0;
					return;}
			case 1: { // actuator select button
					while (flag2.button_pressed ==1) button_scan();
					delay_ms(10);
					target_act++;
					target_act = target_act%4;
					LED_status.aux=target_act ^ 0xFF ;
					display_LED();
					break;}
			case 3: { // move east
					flag2.allow_manual_move_act=1;
					move_act(10,4500,move_act_time_out,0,target_act);
					flag2.allow_manual_move_act=0;
					break; }
			case 4: { // move safty
				    while (flag2.button_pressed ==1) button_scan();
					delay_ms(10);
					tick = (int32)act_full_stroke_tick[target_act]*(int32)(act_safety_stroke-act_min_stroke);
					tick = tick/(act_max_stroke-act_min_stroke);
					target_act_position = (unsigned int16) tick;
					disable_interrupts(GLOBAL);
			 		printf("\r\n Act#%d Len=%lu Now=%lu\r\n",target_act, target_act_position, current_act_position[target_act]);
					enable_interrupts(GLOBAL);
					actuator_move_execute(target_act);
					break; }
			case 5: { // move west
					flag2.allow_manual_move_act=1;
					move_act(10,4500,move_act_time_out,1,target_act);
					flag2.allow_manual_move_act=0;
					break; }



			}
			
		}
		}
		
}

//////////////////////////////////////
void main() {
	int16 relay_time;
	int8 act_loop,i;
	init_spi();
    setup_adc_ports(AN0);
    setup_adc(ADC_CLOCK_INTERNAL);
    set_adc_channel(0);
	set_tris_a(0xFF);
	set_tris_b(0xFF);
	set_tris_d(0x00);
	while(read_adc() < 650) {
		 led_status =0xFE;
		 display_LED();
		 delay_ms(2);
		 led_status =0xFF;
		 display_LED();			
		 delay_ms(50);
	} // trap here  until voltage level is good
	delay_ms(200);
	flag2.power=1;
    T1CON = 0b00000111;  // 0b10011011
    enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
	setup_WDT(WDT_OFF);
    enable_interrupts(GLOBAL);
	while(1);
}
