import sys,locale,marshal,math,struct

def main():
    datenum=[];
    zenith=[];
    azimuth=[];
    tracker_angle=[];
    file_pos=0;
    day=0;
    iDay=0;
    morningIndex=[];
    datum=0;
    pi=22/7;
    filename = 'e:\\BKK2010.txt';
    outputfile = 'e:\\BKK2010.bin';
    file = open(filename, 'r');
    ofile = open(outputfile,'wb');
    print ('--- Reading file %s ---\n' % filename);
    while True:
        line = file.readline()
        if not line:
            break
        sline = line.split('   ');
        # Wait for output buffer to drain.
        datenum.append(locale.atof(sline[0]));
        zenith.append(locale.atof(sline[1]));
        azimuth.append(locale.atof(sline[2]));
    print ('--- Finish  ---\n');
    nrow = len(datenum);
    for i in range(0,nrow):
        if ((zenith[i]<=90) & (day==0) ): # if the sun is above horizon
            day=1; #toggle day
            iDay=iDay+1;
            datum=0;
            morningIndex.append(i);
            file_pos=ofile.tell()
            data=struct.pack('I',900*i);
            ofile.write(data);
            ofile.flush();
            if ((ofile.tell() - file_pos) != 4):
                break;
        elif ((zenith[i]>90) & (day==1) ): # if the sun is setting
            day=0; #toggle night
            for j in range(datum,63):
                file_pos=ofile.tell()
                data=struct.pack('I',0);
                ofile.write(data);
                ofile.flush();
                if ((ofile.tell() - file_pos) != 4):
                   break;

            print('Day %d,  %d \r\n'%(iDay,datum));
        if (day==1):
            tracker_angle = math.atan((math.sin(zenith[i]*pi/180)* math.cos((azimuth[i]-90)*pi/180))/math.cos(zenith[i]*pi/180))*180/pi+90;
            data=struct.pack('I',tracker_angle*(2**22)); # actual tracker angle data, max 63 data per page
            file_pos=ofile.tell()
            ofile.write(data);
            ofile.flush();
            if ((ofile.tell() - file_pos) != 4):
                break;
            datum = datum+1;
    ofile.close()




if __name__ == '__main__':
    main()