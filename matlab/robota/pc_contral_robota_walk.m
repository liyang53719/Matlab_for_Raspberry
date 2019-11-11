% set up serialportlist
clear
addpath('.\trajectory_planner')
s = serialport("COM3",115200);
                      
position_zero=[ 1500,   1500,    1500,...
                1500,   1500,    1500,...
                1500,   1500,    1500,...
                1500,   1500,    1500];
            
for i=1:12
    write(s,['#',num2str(i),'P' num2str(position_zero(i)) 'T1000!'],"uint8");
end


% read walk step
[foot_step_x, foot_step_y] = trajectory_planner();

wait_time = 1.5;
while(1)
    serial_walk_control(s,0.0,0.0,0,10,0,-150,1,foot_step_x,foot_step_y);
end

