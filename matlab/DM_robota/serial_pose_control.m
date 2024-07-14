function [position_pos] = serial_pose_control(s,rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z,t)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
position_zero=[ 1500,   1500,    1550,...
                1500,   1500,    1550,...
                1500,   1500,    1550,...
                1500,   1500,    1550];
            
leg_pose = pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z);
lf_x  = (leg_pose(1,1));
lf_y  = (leg_pose(2,1));
lf_z  = (leg_pose(3,1));
lb_x  = (leg_pose(1,2));
lb_y  = (leg_pose(2,2));
lb_z  = (leg_pose(3,2));
rf_x  = (leg_pose(1,3));
rf_y  = (leg_pose(2,3));
rf_z  = (leg_pose(3,3));
rb_x  = (leg_pose(1,4));
rb_y  = (leg_pose(2,4));
rb_z  = (leg_pose(3,4));

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
%     ['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!']
    write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!'],"uint8");
end
end

