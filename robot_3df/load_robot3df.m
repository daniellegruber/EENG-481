% Sampling time
Ts = 0.001; 

% Get rigidTreeBody object from file
% df3_arm has the rigidBodyTree we will use for other stuff
[df3_arm, df3_arm_info] = importrobot("robot_3df.slx"); 

%%
% Waypoints 
t = 0.5:1:4;
q1_signal = deg2rad(5) * sin(pi*t);
q2_signal = deg2rad(10) * sin(pi*t);
q3_signal = deg2rad(15) * sin(pi*t);
wp = [q1_signal; q2_signal; q3_signal];
plot(t, q1_signal)