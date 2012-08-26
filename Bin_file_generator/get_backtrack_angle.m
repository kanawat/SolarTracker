function [bt_angle] = get_backtrack_angle(alpha,tracker)
   ll = tracker.PANEL_WIDTH;
   dd = tracker.ROW_SPACING;
   tan_alpha = tan(alpha*pi/180.0);
   beta = acos(-1*dd./(ll*sqrt(1.0+tan_alpha.*tan_alpha))) -atan(-1.0*tan_alpha);
   bt_angle =  mod(real(beta),pi)*180.0/pi;