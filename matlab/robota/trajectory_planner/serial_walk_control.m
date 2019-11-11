function [position_pos] = serial_walk_control(s,rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z,t,foot_step_x,foot_step_y)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
position_zero=[ 1500,   1500,    1550,...
    1500,   1500,    1550,...
    1500,   1500,    1550,...
    1500,   1500,    1550];
z_gain = 250;
x_gaim = 400;
time_step = 50;
for j = 1:time_step
    leg_pose = pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z);
    lf_x  = (leg_pose(1,1)) - x_gaim*(foot_step_x(200/time_step*j));
    lf_y  = (leg_pose(2,1));
    lf_z  = (leg_pose(3,1)) + z_gain*(foot_step_y(200/time_step*j)-0.23);
    lb_x  = (leg_pose(1,2)) - x_gaim*(foot_step_x(1 + 200/time_step*mod(j+time_step/2,time_step)));
    lb_y  = (leg_pose(2,2));
    lb_z  = (leg_pose(3,2)) + z_gain*(foot_step_y(1 + 200/time_step*mod(j+time_step/2,time_step))-0.23);
    rf_x  = (leg_pose(1,3)) - x_gaim*(foot_step_x(1 + 200/time_step*mod(j+time_step/2,time_step)));
    rf_y  = (leg_pose(2,3));
    rf_z  = (leg_pose(3,3)) + z_gain*(foot_step_y(1 + 200/time_step*mod(j+time_step/2,time_step))-0.23);
    rb_x  = (leg_pose(1,4)) - x_gaim*(foot_step_x(200/time_step*j));
    rb_y  = (leg_pose(2,4));
    rb_z  = (leg_pose(3,4)) + z_gain*(foot_step_y(200/time_step*j)-0.23);
    
    [lf_alfa,lf_beta,lf_gamma]=xyztoang(lf_x,lf_y,lf_z,0,105,130);
    [lb_alfa,lb_beta,lb_gamma]=xyztoang(lb_x,lb_y,lb_z,0,105,130);
    [rf_alfa,rf_beta,rf_gamma]=xyztoang(rf_x,rf_y,rf_z,0,105,130);
    [rb_alfa,rb_beta,rb_gamma]=xyztoang(rb_x,rb_y,rb_z,0,105,130);
    
    %% stand by
    position_angle=[-rf_beta,-rf_alfa,rf_gamma,...
        lf_beta,lf_alfa,lf_gamma,...
        -rb_beta,-rb_alfa,-rb_gamma,...
        lb_beta,lb_alfa,-lb_gamma];
    position_pos = fix(position_angle./pi.*180./135.*1000);
    
    
    for i=1:12
        write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!'],"uint8");
    end
%     pause(0.05);
end
end

