
LEG_LENGTH = 0.282;
MAX_DISPLACEMENT = 0.125;
STANCE_DEPTH = 0.005;
LEG_LENGTH = 0.282;
MAX_FOOT_DISPLACEMENT = 0.125;
[control_points_x, control_points_y] = scale_control_points(LEG_LENGTH, MAX_FOOT_DISPLACEMENT);

swing_x_points = 0;
swing_y_points = 0;

stance_x_points = 0;
stance_y_points = 0;

tp = TrajectoryPlanner_init(MAX_DISPLACEMENT, STANCE_DEPTH);
% #fig = plt.figure()
% #ax = fig.gca(projection='2d')
% plt.ylim([0, 0.3])
% plt.xlim([-0.2, 0.2])
% plt.grid(linestyle='-', linewidth='0.5', color='gray')

for i =1:100
  phase_magnitude = i / 100;
  [x_swing, y_swing] = TrajectoryPlanner_generate_swing(tp,phase_magnitude);
  [x_stance, y_stance] = TrajectoryPlanner_generate_stance(tp,phase_magnitude);
  ref_x = 0.0;
  ref_y = 0.28;
  swing_x_points(i)=(ref_x - x_swing);
  swing_y_points(i)=(ref_y + y_swing);
  stance_x_points(i)=(ref_x - x_stance);
  stance_y_points(i)=(ref_y + y_stance);
end
plot(swing_x_points, swing_y_points)
hold on
plot(stance_x_points, stance_y_points)
