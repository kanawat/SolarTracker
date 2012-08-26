#include <classic_solar_lib.h>
//////////////////////////////////
/*
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
*/

//////////////////////////////////
#int_timer1
void timer1_ovf()
{   // overflow every 1 sec
      //set_timer1(get_timer1()+0x7FB8);
	  #asm
		MOVLW 0x80;
		MOVWF TMR1H;
	  #endasm
      timer_sec+=1;
      last_command +=1;
	  //timer_sec = timer_sec/45;
	  //timer_sec =timer_sec*45; 
      if (timer_sec>=86400) {
		nDay = (nDay%1461)+1;
	  }
	  timer_sec = timer_sec % 86400;
	  if (last_command > 180)// if no command recv in 3 minutes, reset the RS232
	  {	 init_rs232();
		 last_command =0;
		 cmd_len =0;
      }   

/* // ADC for wall power is not available in this version
	  if (read_adc()<650) {
		 if (flag2.is_moving) flag2.abort_current_activity=1;
		 flag2.power = 0;
		 disable_interrupts(INT_RDA);
		 led_status =0xFE;
		 display_LED();
		 delay_ms(10);
		 led_status =0xFF;
		 display_LED();
         return;
	  } else {
		 if (flag2.power==0) {
			enable_interrupts(INT_RDA);
			init_rs232();
		 }
		 flag2.power=1;
	  }
*/
      
      if (flag2.en_operate == 1) {
		flag.update_time = true;
//	  	led_status.power = !led_status.power; // blink power led
		// display morning, noon, evening
//	    if(timer_sec >64800 || timer_sec <21600) {
//			led_status.aux=3;
//		} else if ( timer_sec < 36000 ) {
//			led_status.aux=2;
//		} else if ( timer_sec < 50400) {
//			led_status.aux=0;
//		} else {
//			led_status.aux=1;
//		}

//	  	display_LED();
      	if ((timer_sec%15) ==0) flag.measure_current=true;
	  	switch ((unsigned int16) (timer_sec % 450)) {
			case 0: {	bit_set(actuator_move_mask,0); flag.measure_current = true; break;}
			case 110: {	bit_set(actuator_move_mask,1); break;}
			case 220: {	bit_set(actuator_move_mask,2); break;}
			case 330: {	bit_set(actuator_move_mask,3); break;}
	  	}
	  }
//	  button_scan();
//	  if (nButton == 0) flag2.abort_current_activity=1;

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
     default: 	if ((c>=' ')&&(c<='~')) 
				 if(cmd_len<=MAX_CMD_LEN) {
					cmd_msg[cmd_len++]=c;
				    last_command =0; 
				}
				break; 
   }
}

void process_cmd_msg(){
   int8 i,j;
   flag.cmd_posted =false;
   last_command =0;
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
		 setup_WDT(WDT_OFF);
         printf("\r\n PAGE %ld:",atol(cmd_msg));
         print_page_data(atol(cmd_msg));
		 setup_WDT(WDT_ON);
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
         print_date_time(0x01);
         break;
      }
      case 'w': { //buffer1 read
         disable_interrupts(GLOBAL);
		 printf("\r\n Buffer1:");
    	for(i=0;i<16;i++) {
      		printf("\r\n%02X : ",i*16);
      		for (j=0;j<8;j++) {
        		flash_buffer1_read(i*16+j*2);
				printf("%02X %02X ",flash_page_data,flash_page_data2);
				restart_wdt();
	  	 	}
         }    
         enable_interrupts(GLOBAL);         
         break;	}
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
		 n_avg=0;
		 accumulate_current=0;
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
         move_act(2000,0xFFFF,move_act_time_out,1,atoi(cmd_msg)); // move actuator to west
         move_act(2000,0xFFFF,move_act_time_out,0,atoi(cmd_msg)); // move actuator to home position
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
		 break;
	  }

      case 'y': { //adc conversion
         set_adc_channel(0);
         delay_us(20);
         printf("\r\nADC= %lu",read_adc());
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
/*
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
      		case 2: { //return home
         			move_act(2000,9000,move_act_time_out,1,target_act); // move actuator to west
         			move_act(2000,9000,move_act_time_out,0,target_act); // move actuator to home position
         			act_full_stroke_tick[target_act]= actuator_pulse;
         			current_act_position[target_act] =0;
         			write_eeprom_data(1); // save full_stroke_tick and current_position
         			break;
      				}
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
*/

void log_data() {

	unsigned int16 current_period;
	unsigned int16 sun_rise_period;
	unsigned int16 average_current;
	int8 index_to_write;
	current_period = (timer_sec+225)/450;
    flash_read_page(nDay,0x02); // get sun rise time
	sun_rise_period=make16(flash_page_data2,flash_page_data);
    // write the sunrise time to buffer1
    flash_read_main_memory_to_buffer1(LOGGER_START_PAGE+((nDay-1)%365)+1);
    if (current_period < sun_rise_period) { 
		printf("\r\n too early \r\n");
		return;}
    if (current_period == sun_rise_period) { // start of day, clear the page first
		printf("\r\n star new log \r\n");
		flash_buffer1_write(0x00, 0x00, 255);
	} 
    if (current_period - sun_rise_period < 124) {
		// write day
		average_current = accumulate_current / n_avg;
		accumulate_current =0;
		n_avg=0;
		printf("\r\n log \r\n");
		flash_buffer1_write(flash_page_data, 0x02, 1);
		flash_buffer1_write(flash_page_data2, 0x03, 1);
		flash_buffer1_write(make8(nDay,0), 0x00, 1);
		flash_buffer1_write(make8(nDay,1), 0x01, 1);
		index_to_write = (int8) 4+(current_period-sun_rise_period)*2;
		flash_buffer1_write(make8(average_current,0),index_to_write, 1);
		flash_buffer1_write(make8(average_current,1),index_to_write+1, 1);
		flash_write_buffer1_to_main_memory(LOGGER_START_PAGE+((nDay-1)%365)+1);
 	}  else printf("\r\n too late \r\n"); 
}

//////////////////////////////////////
void main() {
	int16 relay_time;
	int8 act_loop,i;
	init_spi();
    setup_adc_ports(AN0);
    setup_adc(ADC_CLOCK_INTERNAL);
    set_adc_channel(0);
/*

	set_tris_a(0xFF);
	set_tris_b(0xFF);
	set_tris_d(0x00);
*/

   set_tris_d(0x00); // all D are output
   set_tris_e(0b00000010); // RE1 = emergnecy input
   set_tris_a(0b11111111); // A0 -1 is output
   set_tris_b(0xA0);
   set_tris_c(0b10010000);
   delay_ms(1000);
/* // ADC for wall power is not valid here //

	while(read_adc() < 650) {
		 led_status =0xFE;
		 display_LED();
		 delay_ms(2);
		 led_status =0xFF;
		 display_LED();			
		 delay_ms(50);
	} // trap here  until voltage level is good
*/
	delay_ms(200);
	flag2.power=1;
	output_high(PIN_D6); //always on for power control

	porte.tx_en=1;
    lcd_init();


	printf("\r\n init");
	flash_wait_until_ready();
	printf("\r\n read flash");
	flash_read_mfg_id();
	printf("\r\n Mfg:%02X %02X %02X %02X",flash_mfg_id[0],flash_mfg_id[1],flash_mfg_id[2],flash_mfg_id[3]);
	flash_read_stat(); // stat & 0xBF == 0x9C means device ready
	printf("\r\n Stat: %02X",flash_stat);
    // check flash readiness here //
	read_eeprom_data();
	print_fw_info();
    T1CON = 0b00001111;  // 0b10011011

    enable_interrupts(INT_RDA);
    enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
    enable_interrupts(GLOBAL);
	init_rs232();
	relay_time =0;
	flag2.en_operate =1;
	flag2.button_pressed=0;
	flag2.allow_manual_move_act=0;
//	LED_status =0x00;

	delay_ms(200);
	while(read_eeprom((int8)0xf00040)!=0x34);
	while(read_eeprom((int8)0xf00041)!=0x12);
//	display_LED();
	startup_counter++;
	write_eeprom_data(0);
	setup_wdt(WDT_ON);
	while(1) {
	    restart_wdt();
		if (flag.cmd_posted==1) process_cmd_msg();
		if (flag.update_time) {
			flag.update_time=false;
			print_date_time(0x02);
		}

		if (flag.measure_current) {
			flag.measure_current = false;
			accumulate_current += read_adc();
			n_avg++;
		}
		for(act_loop=0;act_loop<1;act_loop++) // limit to work only act 0
		{
			if (bit_test(actuator_move_mask,act_loop)==1){
				log_data(); // save measured current into flash first
				if (act_full_stroke_tick[act_loop]>0x0010) {
         		solar_get_act_length(act_loop);
				disable_interrupts(GLOBAL);
		 		printf("\r\n Act#%d Len=%lu Now=%lu\r\n",act_loop, target_act_position, current_act_position[act_loop]);
				enable_interrupts(GLOBAL);
		 		actuator_move_execute(act_loop);
		 		printf("done \r\n");
		   		bit_clear(actuator_move_mask,act_loop);
				write_eeprom_data(0);
				print_date_time(0x03);
				lcd_init();
			} 
			}
		}

//		if (nButton==0) {
//		    button_menu();
//		}
		restart_wdt();
	}

DEAD_TRAP: while(1);
}
