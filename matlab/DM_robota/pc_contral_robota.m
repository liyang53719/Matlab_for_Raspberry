% set up serialportlist
clear
s = serialport("COM3",115200)
            
%% go to zero 
% position_zero=[ 1500,   1500,    1500,...
%                 1500,   1500,    1500,...
%                 1500,   1500,    1500,...
%                 1500,   1500,    1500];

% position_zero=[ 1500,   2100,    1500,...
%                 1500,   900,    1500,...
%                 1500,   2100,    1500,...
%                 1500,   900,    1500];
            
position_zero=[ 1500,   1500,    1500,...
                1500,   1500,    1500,...
                1500,   1500,    1500,...
                1500,   1500,    1500];
            
for i=1:12
    write(s,['#',num2str(i),'P' num2str(position_zero(i)) 'T1000!'],"uint8");
end

%% stand by
% position_down=[ 500,   -500,    0,...
%                 -500,   500,    0,...
%                 500,   -500,    0,...
%                 -500,   500,    0];
%             
% for i=1:12
%     write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_down(i)) 'T1000!'],"uint8");
% end

%% setup leg angle for silunlition
% [alfa,beta,gamma]=xyztoang(90*sin([0:0.01:4*pi]),10,150,0.60,105,130);
% leg_pose = pose_control(0,0.0,0,0,0,-200);
% lf_x  = (leg_pose(1,1));
% lf_y  = (leg_pose(2,1));
% lf_z  = (leg_pose(3,1));
% lb_x  = (leg_pose(1,2));
% lb_y  = (leg_pose(2,2));
% lb_z  = (leg_pose(3,2));
% rf_x  = (leg_pose(1,3));
% rf_y  = (leg_pose(2,3));
% rf_z  = (leg_pose(3,3));
% rb_x  = (leg_pose(1,4));
% rb_y  = (leg_pose(2,4));
% rb_z  = (leg_pose(3,4));
% 
% [lf_alfa,lf_beta,lf_gamma]=xyztoang(lf_x,lf_y,lf_z,0,105,130);
% [lb_alfa,lb_beta,lb_gamma]=xyztoang(lb_x,lb_y,lb_z,0,105,130);
% [rf_alfa,rf_beta,rf_gamma]=xyztoang(rf_x,rf_y,rf_z,0,105,130);
% [rb_alfa,rb_beta,rb_gamma]=xyztoang(rb_x,rb_y,rb_z,0,105,130);
% 
% %% stand by
% position_angle=[-rf_beta,-rf_alfa,-rf_gamma,...
%                 lf_beta,lf_alfa,lf_gamma,...
%                 -rb_beta,-rb_alfa,-rb_gamma,...
%                 lb_beta,lb_alfa,lb_gamma];
% position_pos = fix(position_angle./pi.*180./135.*1000)     
% 
% for i=1:12
%     write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T500!'],"uint8");
% end

% serial_pose_control(s,0.0,0.0,0,0,0,-2000,1000)
% serial_pose_control(s,-0.8,0,0,10,0,-170,1000)
wait_time = 1.5;
while(1)
    serial_pose_control(s,0.0,0.0,0,10,0,-200,1000)
    pause(wait_time)
    serial_pose_control(s,0.0,0.0,0,10,0,-150,1000)
    pause(wait_time)
    % stand
    serial_pose_control(s,0.0,0.0,0,0,0,-200,1000)
    pause(wait_time)
    % qian
    serial_pose_control(s,0.3,0.0,0,0,0,-170,1000)
    pause(wait_time)
    serial_pose_control(s,0.0,0.0,0.0,0,0,-200,1000)
    pause(wait_time)
    % hou
    serial_pose_control(s,-0.3,0.0,0,0,0,-170,1000)
    pause(wait_time)
    serial_pose_control(s,0.0,0.0,0.0,0,0,-200,1000)
    pause(wait_time)
    % zuo
    serial_pose_control(s,0.0,0.3,0,0,0,-170,1000)
    pause(wait_time)
    serial_pose_control(s,0.0,0.0,0.0,0,0,-200,1000)
    pause(wait_time)
    % you
    serial_pose_control(s,0.0,-0.3,0,0,0,-170,1000)
    pause(wait_time)
    % zheng
    serial_pose_control(s,0.0,0.0,0.0,0,0,-200,1000)
end
% i = 1;
% a=sin(i/200*(2*pi));
% serial_pose_control(s,0.3*a,0.0,0,0*a,0,-200,1000)
% for i = 1:200
%     a=sin(i/200*(2*pi));
% %     pause(0.1)
%     serial_pose_control(s,0.3*a,0.0,0,0*a,0,-200,10)
% end
% 
% for i = 1:200
%     a=0.2*sin(i/100*(2*pi));
%     serial_pose_control(s,0,a,0,10,0,-200,100)
% end
% % 
% % for i = 1:200
% %     a=sin(i/100*(2*pi));
%     serial_pose_control(s,0,0,0,0,50*a,-200,100);
% % end
% 
