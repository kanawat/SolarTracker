
import sys, os, serial, threading, time

class comm:

    def __init__(self,port):
        try:
           self.serial = serial.serial_for_url(port, 38400, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=0.3)
        except:
           self.serial = serial.Serial(port, 38400, parity=serial.PARITY_NONE, rtscts=False, xonxoff=False, timeout=0.3)
        print 'Serial port opened'
        self.debug= 1
        self.in_packet = {'Addr':[],'Type':[],'Data':[]}
        self.actuator_len = [0,0,0,0]
        self.actuator_max_tick = [0,0,0,0]
        self.date =[]
        self.time =[]
        self.epoch = 1262278800
        self.act_min_stroke = []
        self.act_max_stroke = []
        self.act_safety_stroke = []
        self.act_full_stroke_tick = [0,0,0,0]
        self.current_act_position = [0,0,0,0]

    def data_coder(self,cmd_msg):
        ## get command message and target address and generate data stream to be sent thru COM port
        stuffing_mask = 0xFF
        coded_msg = []
        for i in cmd_msg:
            if ((i == 0xA9) or (i==0xA8)):
                stuffing_mask = 0xDF
                coded_msg.append(chr(0xA9))
            else:
                stuffing_mask = 0xFF
            coded_msg.append(chr(i&stuffing_mask))
        return coded_msg
    def data_decoder(self,coded_msg):
        destuffing_mask =0x00
        decoded_msg =[]
        for i in coded_msg:
            if ((ord(i) == 0xA9)):
                destuffing_mask = 0x20
            else:
                decoded_msg.append(ord(i)|destuffing_mask)
                destuffing_mask = 0x00
        return decoded_msg
    def data_packet_assembler(self,addr,cmd,data):
        ## Byte     1   : Flag = 0xA8
        ##          2-3 : Device ID
        ##          4   : Command
        ##          5-6 : Data
        ##          7   : Checksum
        ##          8   : Flag = 0xA8
        data_chunk =[addr>>8,addr&0x00FF,cmd,data>>8,data&0x00FF]
        checksum =0x00
        for i in data_chunk:
            checksum ^= i
        data_chunk.append(checksum)
        packet = self.data_coder(data_chunk)
        packet.append(chr(0xA8))
        packet.insert(0,chr(0xA8))
        return packet
    def data_packet_disassembler(self,data_packet):
        checksum = 0x00
        if(self.is_valid_packet(data_packet)==0):
            return
        output =[]
        data_packet = data_packet.strip('\xA8')
        destuffing_mask = 0x00
        for ch in data_packet:
            if (ch == '\xA9'):
                destuffing_mask = 0x20
            else:
                output.append(ord(ch)|destuffing_mask)
                checksum ^= (ord(ch)|destuffing_mask)
                destuffing_mask = 0x00
        if (checksum != 0x00):
            print 'Checksum error'

        self.in_packet['Addr'] = (output.pop(0)<<8)+(output.pop(0))
        self.in_packet['Type'] = output.pop(0)
        output.pop() ## pop last item
        self.in_packet['Data'] = output
    def clear_in_packet(self):
        self.in_packet = {'Addr':[],'Type':[],'Data':[]}
    def is_valid_packet(self,data_packet):
        if (data_packet[0] != '\xA8' and data_packet[len(data_packet)-1] != '\xA8'):
           return 0## bad packet
        else:
           return 1
    def send_serial_data(self,data,size):
        self.serial.flushInput()
        for i in data:
            self.serial.write(i)
            time.sleep(0.001)
        if (size ==0):
            return
        ans= self.serial.read(size)
        if (len(ans)<2):
            if self.debug:
                print 'No response'
            return
        if(self.is_valid_packet(ans)==0):
            if self.debug:
                print 'Bad response'
        self.data_packet_disassembler(ans)
    def receive_serial_data(self,size):
        self.serial.timeout=0.3
        ans= self.serial.read(size)
        if (len(ans)<2):
            if self.debug:
                print 'No response'
            return
        if(self.is_valid_packet(ans)==0):
            if self.debug:
                print 'Bad response'
        self.data_packet_disassembler(ans)
        self.serial.timeout=0.3

    def set_date(self,addr=0xfffe,date=1):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x30,date),256)
        print self.in_packet
    def set_time(self,addr=0xfffe,time1=0):
        time_H = (time1 & 0xFFFF0000) >>16
        time_L = time1 & 0x0000FFFF
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x31,time_L),256)
        print self.in_packet
        self.send_serial_data(self.data_packet_assembler(addr,0x32,time_H),256)
        print self.in_packet
    def get_datetime(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x10,0x00),256)
        if (self.in_packet['Type']!=9):
            return -1
        if self.debug:
            print self.in_packet
        nDay = (self.in_packet['Data'][3] << 24) + (self.in_packet['Data'][2] << 16) + (self.in_packet['Data'][1] << 8) + self.in_packet['Data'][0]
        nTime = (self.in_packet['Data'][7] << 24) + (self.in_packet['Data'][6] << 16) + (self.in_packet['Data'][5] << 8) + self.in_packet['Data'][4]
        current_time= (self.epoch+ (nDay-1)*86400 + nTime)
        if self.debug:
            print time.asctime(time.localtime(current_time))
        return current_time

    def get_actuator_len(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x24,0x00),256)
        if (self.in_packet['Type'] != 4):
            return;
        self.act_min_stroke={}
        self.act_min_stroke[0]=(self.in_packet['Data'][1]<<8)+self.in_packet['Data'][0]
        self.act_min_stroke[1]=(self.in_packet['Data'][3]<<8)+self.in_packet['Data'][2]
        self.act_min_stroke[2]=(self.in_packet['Data'][5]<<8)+self.in_packet['Data'][4]
        self.act_min_stroke[3]=(self.in_packet['Data'][7]<<8)+self.in_packet['Data'][6]

        #self.act_min_stroke = (self.in_packet['Data'][1]<<8)+self.in_packet['Data'][0]
        #self.act_max_stroke = (self.in_packet['Data'][3]<<8)+self.in_packet['Data'][2]
        self.act_max_stroke={}

        self.act_max_stroke[0]=(self.in_packet['Data'][9]<<8)+self.in_packet['Data'][8]
        self.act_max_stroke[1]=(self.in_packet['Data'][11]<<8)+self.in_packet['Data'][10]
        self.act_max_stroke[2]=(self.in_packet['Data'][13]<<8)+self.in_packet['Data'][12]
        self.act_max_stroke[3]=(self.in_packet['Data'][15]<<8)+self.in_packet['Data'][14]

        self.act_safety_stroke = (self.in_packet['Data'][17]<<8)+self.in_packet['Data'][16]

        self.act_full_stroke_tick[0] = (self.in_packet['Data'][19]<<8)+self.in_packet['Data'][18]
        self.act_full_stroke_tick[1] = (self.in_packet['Data'][21]<<8)+self.in_packet['Data'][20]
        self.act_full_stroke_tick[2] = (self.in_packet['Data'][23]<<8)+self.in_packet['Data'][22]
        self.act_full_stroke_tick[3] = (self.in_packet['Data'][25]<<8)+self.in_packet['Data'][24]

        self.current_act_position[0] = (self.in_packet['Data'][27]<<8)+self.in_packet['Data'][26]
        self.current_act_position[1] = (self.in_packet['Data'][29]<<8)+self.in_packet['Data'][28]
        self.current_act_position[2] = (self.in_packet['Data'][31]<<8)+self.in_packet['Data'][30]
        self.current_act_position[3] = (self.in_packet['Data'][33]<<8)+self.in_packet['Data'][32]
        if self.debug:
            print ("Min Stroke = %s cm"%([self.act_min_stroke[k]/256.0 for k in self.act_min_stroke]))
            print ("Max Stroke = %s cm"%([self.act_max_stroke[k]/256.0 for k in self.act_max_stroke]))
            print ("Safety Stroke = %f cm"%(self.act_safety_stroke/256.0))
            print ("ACT0 = %4d/%4d"%(self.current_act_position[0],self.act_full_stroke_tick[0]))
            print ("ACT1 = %4d/%4d"%(self.current_act_position[1],self.act_full_stroke_tick[1]))
            print ("ACT2 = %4d/%4d"%(self.current_act_position[2],self.act_full_stroke_tick[2]))
            print ("ACT3 = %4d/%4d"%(self.current_act_position[3],self.act_full_stroke_tick[3]))


    def set_max_act_len(self,addr=0xfffe,actuator=0,length=123.5):
        length = int(length*256)
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x28,(actuator&0x0003)|(length&0xFFFC)),256)
        if self.debug:
            print self.in_packet

    def set_min_act_len(self,addr=0xfffe,actuator=0,length=123.5):
        length = int(length*256)
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x27,(actuator&0x0003)|(length&0xFFFC)),256)
        if self.debug:
            print self.in_packet



    def move_actuator_east(self,addr=0xfffe,actuator=0,pulse=1):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x20,pulse|(actuator<<14)),256)
        if self.debug:
            print self.in_packet
    def move_actuator_west(self,addr=0xfffe,actuator=0,pulse=1):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x21,pulse|(actuator<<14)),256)
        if self.debug:
            print self.in_packet
    def go_home(self,addr=0xfffe,actuator=0):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x22,actuator),256)
        if self.debug:
            print self.in_packet
    def cal_actuator(self,addr=0xfffe,actuator=0):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x23,actuator),256)
        if self.debug:
            print self.in_packet
    def actuator_move_execute(self,addr=0xfffe,actuator=0):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x25,actuator),256)
        if self.debug:
            print self.in_packet
            print("%s"%(self.in_packet['Data'][1]*256+self.in_packet['Data'][0]))
    def get_status_flag(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x11,0),256)
        if self.debug:
            print self.in_packet
    def get_actuator_pulse(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x26,0),256)
        if self.debug:
            print self.in_packet
    def read_flash_to_buffer1(self,addr=0xfffe,page=0):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x83,page),256)
        if self.debug:
            print self.in_packet
    def read_buffer1(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x84,0),512)
        return self.in_packet
    def flash_block_erase(self,addr=0xfffe):
        self.clear_in_packet()
        self.serial.timeout=10
        self.send_serial_data(self.data_packet_assembler(addr,0x80,0),256)
        self.serial.timeout=1
        if self.debug:
            print self.in_packet
    def update_time(self,addr=0xfffe):
        current_time= (time.time()-self.epoch)
        nDay = (current_time/86400)%1461+1
        nTime = (current_time%86400)
        self.set_time(addr,int(nTime))
        self.set_date(addr,int(nDay))

    def send_files(self,addr=0xfffe,filename=''):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x02,00),256)
        print self.in_packet


        try:
            file = open(filename, 'rb')
            file.seek(0,2) # seek to the end of file
            file_size = file.tell()
            sys.stderr.write('--- Sending file %s ---\n' % filename)
            sys.stderr.write('--- checking file size %d bytes, %f blocks ---\n' %(file_size, file_size/256.0))
            nBlock = file_size/256
            byte_counter = 0
            file.seek(0,0) # go to the start of the file
            for i in range(nBlock):
                print i
                self.clear_in_packet()
                self.serial.flushInput()
                self.send_serial_data(self.data_packet_assembler(addr,0x8F,i),0) ## not wait for return packet
                time.sleep(0.03);
                line = file.read(256)
                if not line:
                     break
                self.serial.write(line)
                time.sleep(0.03)
                self.receive_serial_data(256)
                print self.in_packet
                if (self.in_packet['Type'] != 8):
                    sys.stderr.write('\n--- Bad response from target ---\n')
                    break
            sys.stderr.write('\n--- File %s sent ---\n' % filename)
        except IOError, e:
            sys.stderr.write('--- ERROR opening file %s: %s ---\n' % (filename, e))
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x02,01),256) ## turn on normal operation
        print self.in_packet

    def set_256_page_size(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x8E,00),256)
        print self.in_packet
    def read_device_id(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x13,00),256)
        print self.in_packet
    def write_device_id(self,addr=0xfffe,newaddr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x12,newaddr),256)
        print self.in_packet
    def enable_comm(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x01,1),256)
        if self.debug:
            print self.in_packet
        if (self.in_packet['Type'] != 0):
            return 0
        else:
            return 1
    def disable_comm(self,addr=0xfffe):
        self.clear_in_packet()
        self.send_serial_data(self.data_packet_assembler(addr,0x01,0),256)
        if self.debug:
            print self.in_packet
    def scan_active_device(self, addr):
        found_list =[]
        for i in addr:
            self.disable_comm()
            self.enable_comm(i)
            if self.in_packet['Addr'] == i:
                print("Found Device:%d"%i)
                found_list.append(i);
            self.disable_comm()
        return found_list




##
##
##

##    def buffer1_read():
##
##    def get_status():
##    def get_fw_info():
##    def align_actuator():
##    def go_home():
##    def set_operation(): ## normal / halt





##kk=soltracker_comm('/dev/ttyUSB0')
##kk=soltracker_comm('COM3')
##kk.set_date(0xFFFE,0xA8)
##kk.set_time(0xFFFE,12*3600)
##kk.get_datetime(0xFFFE)
##kk.flash_block_erase(0xFFFE)
##kk.get_datetime(0xFFFE)
##kk.read_device_id(0xFFFE)
##kk.write_device_id(0xFFFE,0x0406)
##kk.read_device_id(0xFFFE)

##kk.serial.close()

