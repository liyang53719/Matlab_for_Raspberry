function [control_points_x,control_points_y] = scale_control_points(leg_length , max_foot_displacement)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% def scale_control_points(leg_length, max_foot_displacement):
  control_points_x = [-0.15, -0.2805,-0.3,-0.3,-0.3,   0.0, 0.0 ,   0.0, 0.3032, 0.3032, 0.2826, 0.15];
  control_points_y = [ 0.5,  0.5, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.5, 0.5];
  total_control_points = length(control_points_x);
  leg_ratio = leg_length / 0.444;

  for i =1:12
      if (i == 0)
          control_points_x(i) = -max_foot_displacement / 2.0;
      else if (i == 11)
              control_points_x(i) = max_foot_displacement / 2.0;
          else
              control_points_x(i) = control_points_x(i) * leg_ratio;
          end
      end
      control_points_y(i) = (control_points_y(i) * leg_ratio) - (0.5 * leg_ratio);
  end
  
end

