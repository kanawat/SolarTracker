function [act_len] =  actuator_length(sun_angle,tracker) 

alpha = atan(tracker.DIM_B/tracker.DIM_K);
sin_beta = sin((sun_angle*pi/180.0)-alpha);
cos_beta = cos((sun_angle*pi/180.0)-alpha);
tan_beta = tan((sun_angle*pi/180.0)-alpha);

len_c = sqrt(tracker.DIM_K^2+tracker.DIM_B^2)-tracker.DIM_P/sin_beta;
temp1 = tracker.DIM_A - (tracker.DIM_P/tan_beta)-len_c*cos_beta;
len_l = len_c*sin_beta*len_c*sin_beta+temp1^2-tracker.DIM_M^2;
len_l = sqrt(len_l);

act_len = len_l;
