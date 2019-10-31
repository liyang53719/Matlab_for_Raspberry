function [x,y] = TrajectoryPlanner_generate_swing(self, phase_signal)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明                   
%     def generate_swing(self, phase_signal):
        n = self.total_control_points;
        x = 0;
        y = 0;

        for i =1:self.total_control_points
            coeff = self.factorial(n) / (self.factorial(i) * self.factorial(n - i+1));

%             x = x + (coeff * pow(phase_signal, i) * pow((1 - phase_signal), (n - i)) * self.control_points_x(i));
%             y = y + (coeff * pow(phase_signal, i) * pow((1 - phase_signal), (n - i)) * self.control_points_y(i));
            x = x + (coeff * (phase_signal^(i-1)) * ((1 - phase_signal)^(n - i)) * self.control_points_x(i));
            y = y + (coeff * (phase_signal^(i-1)) * ((1 - phase_signal)^(n - i)) * self.control_points_y(i));
        end
%         return x, y

%     def generate_stance(self, phase_signal):
%         x = 0
%         y = 0
%         x = (self.max_foot_displacement / 2) * (1 - (2 * phase_signal)) 
%         y = self.stance_depth * math.cos((3.1416 * x) / self.max_foot_displacement)
%         return x, y
end

