%%%%%%%%%% define tracker geometry here %%%%%%%%%%%%%%%%%%%
tracker = [];
tracker.DIM_A = 101.0;
tracker.DIM_P = 9.3;                
tracker.DIM_M = 5.5;                
tracker.DIM_K =30.0;                
tracker.DIM_B =0.0;                 
tracker.MIN_STROKE = 76.0;                 
tracker.MAX_STROKE = 123.5;                
tracker.ROW_SPACING =300.00;                
tracker.PANEL_WIDTH =150.00;     
UTC = 7.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


b=[];
b.latitude =13.734049;
b.longitude = 100.662231;
b.altitude = 20;
filename = 'e:\\BKK2010_t2.bin';
ff=fopen(filename,'wb');
znt=[];
azm=[];
tracking_angle=[];
rdchecksum=0;
%% write information about location & tracker configuration on page 0 (256 bytes)%%
fwrite(ff,'SolTracker V2.0 ','char'); %  byte 0x00-0x0F
fwrite(ff,'Bangkok         ','char'); %  byte 0x10-0x1F
fwrite(ff,'Thailand        ','char'); %  byte 0x20-0x2F
fwrite(ff,datestr(now,'dd-mmm-yyyy HH:MM:SS'),'char');     %  byte 0x30-0x44 , generated date, size = 20 bytes
fwrite(ff,uint16(tracker.DIM_A*256),'uint16'); % 0x44
fwrite(ff,uint16(tracker.DIM_P*256),'uint16'); % 0x46
fwrite(ff,uint16(tracker.DIM_M*256),'uint16'); % 0x48
fwrite(ff,uint16(tracker.DIM_K*256),'uint16'); % 0x4A
fwrite(ff,uint16(tracker.DIM_B*256),'uint16'); % 0x4C
fwrite(ff,uint16(tracker.MIN_STROKE*256),'uint16'); %0x4E
fwrite(ff,uint16(tracker.MAX_STROKE*256),'uint16'); %0x50
fwrite(ff,uint16(tracker.ROW_SPACING*100),'uint16'); %0x52
fwrite(ff,uint16(tracker.PANEL_WIDTH*100),'uint16'); %0x54
fwrite(ff,uint16(b.latitude*100),'uint16'); % 0x56
fwrite(ff,uint16(b.longitude*100),'uint16'); % 0x58
fwrite(ff,uint16(b.altitude*100),'uint16'); % 0x5A
fwrite(ff,uint16(actuator_length(90,tracker)*256),'uint16'); %0x5C
for j=hex2dec('5E'):255
    fwrite(ff,255,'uint8'); % zero filled 
end


%%%%%%%%%%%%%%%%%%
for nDay = 1:1461
    sun_rise_flag=0;
    nData=0;
    disp(['Day:' num2str(nDay)]);            
    fwrite(ff,uint16(nDay),'uint16');
    nData=nData+1;
    for nTime = 0:192
        [ znt,azm ]=sun_position2(nDay+(nTime/192.0)-(UTC/24.0),b);        
        if (sun_rise_flag==0 && znt <=90) % sunrise
            sun_rise_flag=1;
            fwrite(ff,uint16(nTime),'uint16');
            nData=nData+1;
        end
        tracking_angle = atan(sin(znt*pi/180).*cos(azm*pi/180-pi/2)./cos(znt*pi/180));
        tracking_angle = (pi/2-tracking_angle)*180/pi;
        if (sun_rise_flag==1 && znt <=90 ) % midday
            if (tracking_angle <30 && tracking_angle >0)
                fwrite(ff,uint16(actuator_length(90-get_backtrack_angle(90-tracking_angle,tracker),tracker)*256),'uint16');
                nData=nData+1;
            elseif (tracking_angle <180 && tracking_angle >150)
                fwrite(ff,uint16(actuator_length(90+get_backtrack_angle(tracking_angle-90,tracker),tracker)*256),'uint16');
                nData=nData+1;
            else
                fwrite(ff,uint16(actuator_length(tracking_angle,tracker)*256),'uint16');
                nData=nData+1;              
            end 
        end
        if (sun_rise_flag==1 && znt > 90) % sunset 
            for j=(nData):127
              fwrite(ff,uint16(actuator_length(90,tracker)*256),'uint16'); % zero filled 
            end
            break;
            
        end
    end
end
fclose(ff);
disp('calculate read checksum');
ff=fopen(filename,'r+');
if(fseek(ff,256,-1)==0) % seek to page 1
    while(1)
        raw_data = fread(ff,1,'uint16');
        if (isempty(raw_data))
            break;
        end
        rdchecksum = uint16(bitxor(uint16(rdchecksum),uint16(raw_data)));
    end
end
disp(dec2hex(rdchecksum))
fseek(ff,254,-1); % seek to the end of page 0
fwrite(ff,uint16(rdchecksum),'uint16');
fclose(ff);






    