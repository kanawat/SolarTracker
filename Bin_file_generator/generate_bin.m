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


i=[0:1/(24*8):1461]-(UTC/24); % generate data for 4 years (including leap year) .. collecting 12 data point in 1 hr
b=[];
b.latitude =13.734049;
b.longitude = 100.662231;
b.altitude = 20;
date_string =  (i);  % string of 1 year time since 1-jan-2010
sun_zenith=zeros(length(date_string),1);
sun_azimuth=zeros(length(date_string),1);
tracking_angle=zeros(length(date_string),1);
iDay=0;
DayIndex=[];
Day=0;
sunData=0;
filename = 'e:\\BKK2010_2.bin';
ff=fopen(filename,'wb');
znt=[];
azm=[];
tag=[];
for nDay = 1:1461
    sun_rise_flag=0;
    for nTime = 0:192
        [ znt,azm ]=sun_position2(nDay+(nTime/192.0)-(UTC/24.0),b);
        tag = atan(sin(znt*pi/180).*cos(azm*pi/180-pi/2)./cos(znt*pi/180));
        if (sun_rise_flag==0 && znt <=90)
            sun_rise_flag=1;
            disp(['Day:' num2str(nDay) ' Sun rise at:' num2str(nTime/8.0) 'am']);
        end
    end
end


for i=1:length(date_string)
   [ sun_zenith(i),sun_azimuth(i)]=sun_position2(date_string(i),b);
   tracking_angle(i)=atan(sin(sun_zenith(i)*pi/180).*cos(sun_azimuth(i)*pi/180-pi/2)./cos(sun_zenith(i)*pi/180));
   if (sun_zenith(i)<=90 && (Day==0))
       Day = 1; % toggle day
       iDay=iDay+1;
       DayIndex(iDay)=i;
       sunData=0;
       fwrite(ff,uint32(i),'uint32');
   elseif (sun_zenith(i)>90 && (Day==1))
       Day =0;
       disp(['Day ' num2str(iDay) ' Data point=' num2str(sunData)]);
       for (j=(sunData+4):127)
           fwrite(ff,0,'uint8'); % zero filled 
       end
   end

   if(sun_zenith(i)<=90)
       sunData =sunData+1;
       fwrite(ff,uint8(round((pi/2-tracking_angle(i))*180/pi)),'uint8');
   end 
end

fclose(ff);



    