#define NETWORK_COMM 1
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
     #asm
      MOVLW 0x00;
      MOVWF TMR1;
      MOVLW 0x80;
      MOVWF TMR1H;
     #endasm
      timer_sec+=1; 
     last_command +=1;
      if (timer_sec>=86400) {
      nDay = (nDay%1461)+1;
     }
     timer_sec = timer_sec % 86400;
     if (last_command > 180)// if no command recv in 3 minutes, reset the RS232
     {    init_rs232();
       last_command =0;
       cmd_len =0;
      }   

     if ((read_adc()+read_adc()+read_adc()+read_adc())/4<600) {
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

      flag.update_time = true;
      if (flag2.en_operate == 1) {
        led_status.power = !led_status.power; // blink power led
      // display morning, noon, evening
       if(timer_sec >64800 || timer_sec <21600) {
         led_status.aux=3;
      } else if ( timer_sec < 36000 ) {
         led_status.aux=2;
      } else if ( timer_sec < 50400) {
         led_status.aux=0;
      } else {
         led_status.aux=1;
      }

        display_LED();
        switch ((unsigned int16) (timer_sec % 450)) {
         case 0: {   bit_set(actuator_move_mask,0); break;}
         case 110: {   bit_set(actuator_move_mask,1); break;}
         case 220: {   bit_set(actuator_move_mask,2); break;}
         case 330: {   bit_set(actuator_move_mask,3); break;}
        }
     }
     button_scan();
     if (nButton == 0) flag2.abort_current_activity=1;

}

void init_spi() {
   setup_spi(spi_master |spi_h_to_l | spi_clk_div_16 );
    SSPSTAT = 0xC0;
}

#int_rda
void recive_cmd()
{
   char c;
   int8 i;
   int8 checksum =0x00;
   int16 msg_addr;

   c=getc();
   switch (c) {   
    case(0xA8):{ if(cmd_len>=4) {
                  for(i=0;i<cmd_len;i++)
                  {
                  checksum ^= cmd_msg[i];
                  }
               if (checksum != 0x00) {cmd_len=0; return;}
               msg_addr = make16(cmd_msg[0],cmd_msg[1]); // address
               if (msg_addr != 0xFFFE && msg_addr != dev_id) {cmd_len=0; return;} // address not correct, just ignore this command
               command_byte = cmd_msg[2];
               aux_command = make16(cmd_msg[3],cmd_msg[4]);
               if (flag2.is_moving) send_data(7,0); // send BUSY
               else flag.cmd_posted=true;
                
             }       
                cmd_len =0;
             break;}
    case(0xA9): {de_stuffing_mask = 0x20; break;}
     default:     {if(cmd_len<=MAX_CMD_LEN) {
               cmd_msg[cmd_len++]=c|de_stuffing_mask;
               last_command =0; 
            }
            de_stuffing_mask = 0x00;
            break; }
   }
}





void process_cmd_msg() {
   int8 i,j;
    int16 temp_mem;
    flag.cmd_posted =false;
    last_command =0;    
    switch(command_byte) {
// level 8 is for flash related
      // 0x80 = block erase
      case 0x80: { flash_block_erase();  send_data(0,0); break; }
      // 0x82  = write buffer 1 to flash page
     case 0x82: { disable_interrupts(GLOBAL);
                  flash_write_buffer1_to_main_memory(aux_command);
               send_data(0,0);
                  enable_interrupts(GLOBAL);
                  break; 
             }
     // 0x83 = read page to buffer 1
     case 0x83: {
                  disable_interrupts(GLOBAL);
                  flash_read_main_memory_to_buffer1(aux_command);
                 send_data(0,0);
                  enable_interrupts(GLOBAL);   
                break;
              }   
     // 0x84 = Buffer1 read
     case 0x84: { //buffer1 read
                  disable_interrupts(GLOBAL);
                send_buffer1_content();
                  enable_interrupts(GLOBAL);         
                  break;   }

     case 0x85: { //test code
                  disable_interrupts(GLOBAL);
               flash_buffer1_write(0,0,255);
               flash_buffer1_write(0x55,0x10,0x08);
                  enable_interrupts(GLOBAL);         
                  break;   }


     // 0x8E = set flash page size = 256
      case 0x8E: {  flash_set_256_page_size(); send_data(6,0); break; }
     // 0x8F = flash write page
     case 0x8F: {
                  disable_interrupts(GLOBAL);
                setup_WDT(WDT_OFF);
                  flash_write_page(aux_command);
               memcpy(output_buffer,&aux_command,2);
                send_data(8,1);
                setup_WDT(WDT_ON);
                  enable_interrupts(GLOBAL);
                  break;}     
// level 3 = date and time setup
     case 0x30: { // set date
         nDay= aux_command;  send_data(0,0);break;    }
     case 0x31: { // set low byte of time
       timer_sec = aux_command; // rouding to 60 sec
       send_data(0,0);
         break;    }
     case 0x32: { // set high byte of time
         temp_mem = &timer_sec;
       memcpy(temp_mem+2,&aux_command,2);
       send_data(0,0);
         break;    }
// level 2 = actuator control
      // 0x20 = move east
     case 0x20: { if (flag2.is_moving) {send_data(1,0); break;}
        send_data(0,0); 
      move_act(aux_command&(0x3FFF),9000,move_act_time_out,1,(int8)(aux_command>>14)); 
       last_actuator_pulse = actuator_pulse;
      break;    }
     // 0x21 = move west
     case 0x21: {  if (flag2.is_moving) {send_data(1,0); break;}
      send_data(0,0);
      move_act(aux_command&(0x3FFF),9000,move_act_time_out,0,(int8)(aux_command>>14)); 
       last_actuator_pulse = actuator_pulse;
      break;    }
     // 0x22 = go home
      case 0x22: {  if (flag2.is_moving) {send_data(1,0); break;}
               send_data(0,0);
               tick = (int32)act_full_stroke_tick[(int8) aux_command]*(int32)(act_safety_stroke-act_min_stroke);
               tick = tick/(act_max_stroke-act_min_stroke);
               target_act_position = (unsigned int16) tick;
               actuator_move_execute((int8) aux_command);
                   last_actuator_pulse = actuator_pulse;
               break; }
     // 0x23 = cal act
     case 0x23: {  if (flag2.is_moving) {send_data(1,0); break;}
               send_data(0,0);
                  move_act(2000,9000,move_act_time_out,1,(int8) aux_command); // move actuator to west
                  move_act(2000,9000,move_act_time_out,0,(int8) aux_command); // move actuator to home position
                  act_full_stroke_tick[(int8) aux_command]= actuator_pulse;
                  current_act_position[(int8) aux_command] =0;
                  write_eeprom_data(1); // save full_stroke_tick and current_position
                last_actuator_pulse = actuator_pulse;
                  break; }
     // 0x24 = report actuator stat
     case 0x24: {  
               memcpy(output_buffer,&act_min_stroke,2);
               memcpy(output_buffer+2,&act_max_stroke,2);
               memcpy(output_buffer+4,&act_safety_stroke,2);
               memcpy(output_buffer+6,act_full_stroke_tick,8);
               memcpy(output_buffer+14,current_act_position,8);
               send_data(4,21);
               break;
              }   
      // 0x25 = execute move on actuator to target position
     case 0x25: {   if (flag2.is_moving) {send_data(1,0); break;}
               send_data(0,0);
                  if (act_full_stroke_tick[(int8) aux_command] > 0x10) {
                  solar_get_act_length((int8) aux_command);
                actuator_move_execute((int8) aux_command);
               }
                last_actuator_pulse = actuator_pulse;
                  break;    }

     case 0x26: { // report last_actuator_move 
               memcpy(output_buffer,&last_actuator_pulse,2);
               send_data(5,1);
                break;
            }   
 
// level 1 command is for generic status
     case 0x10: { memcpy(output_buffer,&nDay,4);
                   memcpy(output_buffer+4,&timer_sec,4);
               send_data(9,7);
               break; }
      // report status flag
     case 0x11: { memcpy(output_buffer,&flag,1);
                   memcpy(output_buffer+1,&flag2,1);
               send_data(10,1);
               break; }
     case 0x12: { //write device id
               dev_id = aux_command;
               write_device_id();
               send_data(0,0);
               break;
               }
      case 0x13: { // read device_id
               read_device_id();
               memcpy(output_buffer,&dev_id,2);
               send_data(11,1);
               break; 
             }


// level 0 communication control
     case 0x01: {  // close/open communication port
                if (aux_command ==0x0000) { 
                  output_low(TX_EN);
               } else if (aux_command ==0x0001) {
                  output_high(TX_EN);
                  delay_us(500);
                  send_data(0,0); // send ack         
               } else send_data(1,0); // send NACK
               break;
             }   

     case 0x02: {  // close/open communication port
                if (aux_command ==0x0000) { 
                  flag2.en_operate=0;
                  send_data(0,0); // send ack         
               } else if (aux_command ==0x0001) {
                  flag2.en_operate=1;
                  send_data(0,0); // send ack         
               } else send_data(1,0); // send NACK
               break;
             }   



   }

}

/*
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
        setup_WDT(WDT_OFF);
         memcpy(cmd_msg,cmd_msg+1,18);
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
         print_date_time();
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
         break;   
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
       break;
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

*/
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
   while(flag2.button_pressed ==1)   button_scan();
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
               init_rs232();
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
   int16 relay_time,kk;
   int8 act_loop,i;
   init_spi();
    setup_adc_ports(AN0);
    setup_adc(ADC_CLOCK_INTERNAL);
    set_adc_channel(0);
   set_tris_a(0xFF);
   set_tris_b(0xFF);
   set_tris_d(0x00);
    led_status =0xF0;
   display_LED();
   delay_ms(5000);
   flag2.is_moving =0;
   while((read_adc()+read_adc()+read_adc()+read_adc())/4 < 650) {
       led_status =0xFE;
       display_LED();
       delay_ms(2);
       led_status =0xFF;
       display_LED();         
       delay_ms(50);
   } // trap here  until voltage level is good
   delay_ms(200);
   flag2.power=1;


   output_low(TX_EN);


   flash_wait_until_ready();
   flash_read_mfg_id();
   flash_read_stat(); // stat & 0xBF == 0x9C means device ready
    // check flash readiness here //
    //if (flash_mfg_id[1]==0x24) //4MBit
    //if (flash_mfg_id[1]==0x26) //16MBit

   read_eeprom_data();
   read_device_id();
   //print_fw_info();
    T1CON = 0b00001111;  // 0b10011011
    enable_interrupts(INT_RDA);
    enable_interrupts(INT_TIMER1);   // Setup interrupt on falling edge
    enable_interrupts(GLOBAL);
   init_rs232();
   relay_time =0;
   flag2.en_operate =1;
   flag2.button_pressed=0;
   flag2.allow_manual_move_act=0;
   LED_status =0x00;

   delay_ms(200);
   while(read_eeprom((int8)0xf00040)!=0x34);
   while(read_eeprom((int8)0xf00041)!=0x12);
   display_LED();
   startup_counter++;
   write_eeprom_data(0);
   solar_load_parameter_from_flash();
   setup_wdt(WDT_ON);

   while(1) {
      if (flag.cmd_posted==1) process_cmd_msg();
      if (bit_test(RCSTA,1)==1) init_rs232();
      for(act_loop=0;act_loop<4;act_loop++)
      {
         if (bit_test(actuator_move_mask,act_loop)==1){
            if (act_full_stroke_tick[act_loop]>0x0010) {
               solar_get_act_length(act_loop);
             actuator_move_execute(act_loop);
               bit_clear(actuator_move_mask,act_loop);
            write_eeprom_data(0);
            //print_date_time();
         } 
         }
      }

      if (nButton==0) {
          button_menu();
      }
      restart_wdt();
        kk = read_adc();
       if ((read_adc()+read_adc()+read_adc()+read_adc())/4 < 600) sleep();
   }

DEAD_TRAP: while(1);
}
