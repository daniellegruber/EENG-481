% Sampling time
Ts = 0.001; 

% Get rigidTreeBody object from file
% df2_arm has the rigidBodyTree we will use for other stuff
[df2_arm, arm_info] = importrobot("test_robot.slx"); 

% Waypoints 
wp = [0.35 0.25 0.25 0.15 0.15 0.25;
    0 0.01 0.11 0.11 0.01 0.01;
    0.11 0.11 0.11 0.11 0.11 0.11];