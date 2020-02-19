% Sel = 1 for single shooting. Sel = 2 for multiple shooting. 
% Sel = 3 for Obstacle avoidance without wall constraint.
% Sel = 4 for Obstacle avoidance with wall constraint.
sel=3;

if sel==1
   clc;clear;
   xs = [20 ;20 ; pi]; % Reference pose.
   mpc_car;
elseif sel==2
   clc;clear;
   xs = [20 ;20 ; pi/2]; % Reference pose.
   mpc_car_ms;
elseif sel==3
   clc;clear;
   xs = [20 ;20 ; pi/2]; % Reference pose.
   mpc_car_NoWall;
else
   clc;clear;
   xs = [20 ;20 ; pi/2]; % Reference pose.
   mpc_car_obs;
end