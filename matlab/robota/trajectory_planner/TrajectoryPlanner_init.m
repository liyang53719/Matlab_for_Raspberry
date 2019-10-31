function self = TrajectoryPlanner_init(max_foot_displacement,stance_depth)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
% class TrajectoryPlanner:
%     def __init__(self, max_foot_displacement, stance_depth):

        self.max_foot_displacement = max_foot_displacement;
        self.stance_depth = stance_depth;
        self.control_points_x = [-0.0625, -0.08415, -0.09, -0.09, -0.09, 0.0, 0.0, 0.0, 0.09096, 0.09096, 0.08478000000000001, 0.0625];
        self.control_points_y = [0.0, 0.0, -0.04167, -0.04167, -0.04167, -0.04167, -0.04167, -0.05357999999999999, -0.05357999999999999, -0.05357999999999999, 0.0, 0.0];
        self.total_control_points = length(self.control_points_x);
        self.factorial =  [1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0];
        
        self.control_points_x(1)  = -self.max_foot_displacement / 2;
        self.control_points_x(self.total_control_points)  = self.max_foot_displacement / 2;
end

