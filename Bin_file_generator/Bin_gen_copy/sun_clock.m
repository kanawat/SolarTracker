
b.latitude =13.734049;
b.longitude = 100.662231;
b.altitude = 20;
ff=figure;

while(1)
    [zenith,azimuth]=sun_position2(etime(clock,[2010 1 1 7 0 0])/86400,b);
    x=sin(zenith*pi/180)*sin(azimuth*pi/180-pi/2);
    y=sin(zenith*pi/180)*cos(azimuth*pi/180-pi/2);
    z=cos(zenith*pi/180);
    plot3(x,y,z,'ro','MarkerSize',30,'MarkerFaceColor',[x y z]);
    grid on;
    %axis([-1 1 -1 1 0 1]);
    hold on;
    pause(10)
end