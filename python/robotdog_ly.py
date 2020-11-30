# -*- coding: utf-8 -*-
import serial
import math
import numpy as np 

import threading
import time
#from mpl_toolkits.mplot3d import Axes3D

def scale_control_points(leg_length, max_foot_displacement):
  control_points_x = [-0.15, -0.2805,-0.3,-0.3,-0.3,   0.0, 0.0 ,   0.0, 0.3032, 0.3032, 0.2826, 0.15]

  control_points_y = [ 0.5,  0.5, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.5, 0.5]
  total_control_points = len(control_points_x)
  leg_ratio = leg_length / 0.444

  for i in range(12):
      if i == 0:
          control_points_x[i] = -max_foot_displacement / 2.0
      elif i == 11:
          control_points_x[i] = max_foot_displacement / 2.0
      else:
          control_points_x[i] = control_points_x[i] * leg_ratio

      control_points_y[i] = (control_points_y[i] * leg_ratio) - (0.5 * leg_ratio)
      
  return control_points_x, control_points_y

#Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
def xyztoang(x,y,z,yoffh,hu,hl):
    dyz=np.sqrt(y**2+z**2)
    lyz=np.sqrt(dyz**2-yoffh**2)
    gamma_yz=-np.arctan(y/z)
    gamma_h_offset=-np.arctan(-yoffh/lyz)
    gamma=gamma_yz-gamma_h_offset
    
    lxzp=np.sqrt(lyz**2+x**2)
    n=(lxzp**2-hl**2-hu**2)/(2*hu)
    beta=-np.arccos(n/hl)
    
    alfa_xzp=-np.arctan(x/lyz)
    alfa_off=np.arccos((hu+n)/lxzp)
    alfa=alfa_xzp+alfa_off
    if any( np.isnan([gamma,alfa,beta])):
        print(x,y,z,yoffh,hu,hl)
    return [alfa,beta,gamma]


class TrajectoryPlanner:
    def __init__(self, max_foot_displacement, stance_depth):
        self.max_foot_displacement = max_foot_displacement
        self.stance_depth = stance_depth
        self.control_points_x = [-0.0625, -0.08415, -0.09, -0.09, -0.09, 0.0, 0.0, 0.0, 0.09096, 0.09096, 0.08478000000000001, 0.0625]
        self.control_points_y = [0.0, 0.0, -0.04167, -0.04167, -0.04167, -0.04167, -0.04167, -0.05357999999999999, -0.05357999999999999, -0.05357999999999999, 0.0, 0.0]
        self.total_control_points = len(self.control_points_x)
        self.factorial =  [1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0]
        
        self.control_points_x[0]  = -self.max_foot_displacement / 2
        self.control_points_x[self.total_control_points - 1]  = self.max_foot_displacement / 2
                    
    def generate_swing(self, phase_signal):
        n = self.total_control_points - 1
        x = 0
        y = 0

        for i in range(self.total_control_points):
            coeff = self.factorial[n] / (self.factorial[i] * self.factorial[n - i])

            x = x + (coeff * pow(phase_signal, i) * pow((1 - phase_signal), (n - i)) * self.control_points_x[i])
            y = y + (coeff * pow(phase_signal, i) * pow((1 - phase_signal), (n - i)) * self.control_points_y[i])

        return x, y

    def generate_stance(self, phase_signal):
        x = 0
        y = 0
        x = (self.max_foot_displacement / 2) * (1 - (2 * phase_signal)) 
        y = self.stance_depth * math.cos((3.1416 * x) / self.max_foot_displacement)
        return x, y

def trajectory_planner():
	LEG_LENGTH = 0.282
	MAX_DISPLACEMENT = 0.125
	STANCE_DEPTH = 0.005

	swing_x_points = []
	swing_y_points = []

	stance_x_points = []
	stance_y_points = []

	tp = TrajectoryPlanner(MAX_DISPLACEMENT, STANCE_DEPTH)
	#fig = plt.figure()
	#ax = fig.gca(projection='2d')
	#plt.ylim([0, 0.3])
	#plt.xlim([-0.2, 0.2])
	#plt.grid(linestyle='-', linewidth='0.5', color='gray')

	for i in range(101):
		phase_magnitude = i / 100
		x_swing, y_swing = tp.generate_swing(phase_magnitude)
		x_stance, y_stance = tp.generate_stance(phase_magnitude)
		print("x_stance","y_stance",x_stance,y_stance)
		ref_x = 0.0
		ref_y = 0.28
		swing_x_points.append(ref_x - x_swing)
		swing_y_points.append(ref_y + y_swing)
		stance_x_points.append(ref_x - x_stance)
		stance_y_points.append(ref_y + y_stance)

	#plt.plot(swing_x_points, swing_y_points)
	#plt.plot(stance_x_points, stance_y_points)

	#plt.show()

#	swing_x_points.append(swing_x_points,stance_x_points)
#	swing_y_points.append(swing_y_points,stance_y_points)
	return swing_x_points, swing_y_points

def pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z):
	# 定义结构参数
	b = 120;
	w = 120;
	l = 250;
	# 定义目标位姿
	#pos = np.mat([0.0,  0.0,  0.3 ]).T # 目标位置向量
#	rpy = np.array([0.0,  0.0,  0.0]) * math.pi / 180 # 欧拉角，化为弧度值
	# 将欧拉角转换为旋转矩阵
#	R, P, Y = rpy[0], rpy[1], rpy[2]
	pos = np.mat([pos_x,  pos_y,  pos_z ]).T

	R = rpy_r;
	P = rpy_p;
	Y = rpy_y;

	rotx = np.mat([[ 1,       0,            0          ],
		 [ 0,       math.cos(R), -math.sin(R)],
		 [ 0,       math.sin(R),  math.cos(R)]])
	roty = np.mat([[ math.cos(P),  0,      -math.sin(P)],
		 [ 0,            1,       0          ],
		 [ math.sin(P),  0,       math.cos(P)]]) 
	rotz = np.mat([[ math.cos(Y), -math.sin(Y),  0     ],
		 [ math.sin(Y),  math.cos(Y),  0     ],
		 [ 0,            0,            1     ]])
	rot_mat = rotx * roty * rotz
	# 结构参数
	body_struc = np.mat([[ l / 2,  b / 2,  0],
		       [ l / 2, -b / 2,  0],
		       [-l / 2,  b / 2,  0],
		       [-l / 2, -b / 2,  0]]).T
	footpoint_struc = np.mat([[ l / 2,  w / 2,  0],
		            [ l / 2, -w / 2,  0],
		            [-l / 2,  w / 2,  0],
		            [-l / 2, -w / 2,  0]]).T
	# 计算单腿末端位置向量AB
	AB = np.mat(np.zeros((3, 4)))
	leg_pose = np.mat(np.zeros((3, 4)))
	position_pos = np.mat(np.zeros((3, 4)))
	for i in range(4):
		leg_pose[:, i] = - pos - rot_mat * body_struc[:, i] + footpoint_struc[:, i]
#	print(AB)

	position_zero=[ 1500,   1500,    1550,
		        1500,   1500,    1550,
		        1500,   1500,    1550,
		        1500,   1500,    1550];
		    
#	leg_pose = AB(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z);
	lf_x  = leg_pose[0,0];
	lf_y  = leg_pose[1,0];
	lf_z  = leg_pose[2,0];
	lb_x  = leg_pose[0,1];
	lb_y  = leg_pose[1,1];
	lb_z  = leg_pose[2,1];
	rf_x  = leg_pose[0,2];
	rf_y  = leg_pose[1,2];
	rf_z  = leg_pose[2,2];
	rb_x  = leg_pose[0,3];
	rb_y  = leg_pose[1,3];
	rb_z  = leg_pose[2,3];

#	print("lf_x",lf_x)	
#	print("lf_y",lf_y)
#	print("lf_z",lf_z)

	[lf_alfa,lf_beta,lf_gamma]=xyztoang(lf_x,lf_y,lf_z,0,105,130);
	[lb_alfa,lb_beta,lb_gamma]=xyztoang(lb_x,lb_y,lb_z,0,105,130);
	[rf_alfa,rf_beta,rf_gamma]=xyztoang(rf_x,rf_y,rf_z,0,105,130);
	[rb_alfa,rb_beta,rb_gamma]=xyztoang(rb_x,rb_y,rb_z,0,105,130);

#	print("lf_alfa",lf_alfa)	
#	print("lf_beta",lf_beta)
#	print("lf_gamma",lf_gamma)
	

	const = 1000*180*3.1416/135;#/3.1415*180/135

	#position_angle=[-1*rf_beta,-rf_alfa,rf_gamma,
	#	        lf_beta,lf_alfa,lf_gamma,
	#	        -rb_beta,-rb_alfa,-rb_gamma,
	#	        lb_beta,lb_alfa,-lb_gamma];
	position_angle=np.mat([	[-1*rf_beta	* const, -rf_alfa *const, rf_gamma  *const],
				[lf_beta	*const , lf_alfa  *const, lf_gamma  *const],
				[-rb_beta	*const , -rb_alfa *const, -rb_gamma *const],
				[lb_beta	*const , lb_alfa  *const, -lb_gamma *const]
			      ]);


#	position_pos = np.int(position_angle*1000/3.1415*180/135)
#	print("const",const)
#	print("position_angle",position_angle)
	position_pos = position_angle;
#	print("position_pos",position_pos)

#	for i in range(12):# i < 11 #12
		#['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!']
		#write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!'],"uint8");
#		position  = position_zero(i) + position_pos(i);
#		send_text = "#"+str(i)+"P"+str(position)+"T100!";
#		result=ser.write(send_text.encode("UTF-8"))
#		print(send_text)
	return leg_pose

def serial_walk_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z,t,foot_step_x,foot_step_y):
#UNTITLED3 此处显示有关此函数的摘要
#   此处显示详细说明
	position_zero=[ 1500,   1500,    1550,
			1500,   1500,    1550,
			1500,   1500,    1550,
			1500,   1500,    1550];
	z_gain = 250;
	x_gaim = 400;
	time_step = 50;
	leg_pose = np.mat(np.zeros((3, 4)))
	for j in range(time_step): #= 1:time_step
		leg_pose = pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z);

#		print("leg_pose", leg_pose)	
#		print("foot_step_x[200/time_step*j]",foot_step_x[200/time_step*j])
#		print("x_gaim*(foot_step_x[200/time_step*j]",x_gaim*(foot_step_x[200/time_step*j]))
#		print("leg_pose[0,3]:", leg_pose[0,3])
#		print("leg_pose[0,1]:", leg_pose[0,1])
#		print("lb_x  = leg_pose[0,1] - x_gaim*(foot_step_x[200/time_step*np.mod(j+time_step/2,time_step)]);",200/time_step*np.mod(j+time_step/2,time_step))
		lf_x  = leg_pose[0,0] - x_gaim*(foot_step_x[200/time_step*j]);
		lf_y  = leg_pose[1,0];
		lf_z  = leg_pose[2,0] + z_gain*(foot_step_y[200/time_step*j]-0.23);
		lb_x  = leg_pose[0,1] - x_gaim*(foot_step_x[200/time_step*np.mod(j+time_step/2,time_step)]);
		lb_y  = leg_pose[1,1];
		lb_z  = leg_pose[2,1] + z_gain*(foot_step_y[200/time_step*np.mod(j+time_step/2,time_step)]-0.23);
		rf_x  = leg_pose[0,2] - x_gaim*(foot_step_x[200/time_step*np.mod(j+time_step/2,time_step)]);
		rf_y  = leg_pose[1,2];
		rf_z  = leg_pose[2,2] + z_gain*(foot_step_y[200/time_step*np.mod(j+time_step/2,time_step)]-0.23);
		rb_x  = leg_pose[0,3] - x_gaim*(foot_step_x[200/time_step*j]);
		rb_y  = leg_pose[1,3];
		rb_z  = leg_pose[2,3] + z_gain*(foot_step_y[200/time_step*j]-0.23);

		#lf_x  = leg_pose[0,0] - x_gaim*(foot_step_x[200/time_step*j]);
		#lf_y  = leg_pose[0,1];
		#lf_z  = leg_pose[0,2] + z_gain*(foot_step_y[200/time_step*j]-0.23);
		#lb_x  = leg_pose[1,0] - x_gaim*(foot_step_x[200/time_step*np.mod(j+time_step/2,time_step)]);
		#lb_y  = leg_pose[1,1];
		#lb_z  = leg_pose[1,2] + z_gain*(foot_step_y[200/time_step*np.mod(j+time_step/2,time_step)]-0.23);
		#rf_x  = leg_pose[2,0] - x_gaim*(foot_step_x[200/time_step*np.mod(j+time_step/2,time_step)]);
		#rf_y  = leg_pose[2,1];
		#rf_z  = leg_pose[2,2] + z_gain*(foot_step_y[200/time_step*np.mod(j+time_step/2,time_step)]-0.23);
		#rb_x  = leg_pose[3,0] - x_gaim*(foot_step_x[200/time_step*j]);
		#rb_y  = leg_pose[3,1];
		#rb_z  = leg_pose[3,2] + z_gain*(foot_step_y[200/time_step*j]-0.23);


		print("lf_x2",lf_x)	
		print("lf_y2",lf_y)
		print("lf_z2",lf_z)

		[lf_alfa,lf_beta,lf_gamma]=xyztoang(lf_x,lf_y,lf_z,0,105,130);
		[lb_alfa,lb_beta,lb_gamma]=xyztoang(lb_x,lb_y,lb_z,0,105,130);
		[rf_alfa,rf_beta,rf_gamma]=xyztoang(rf_x,rf_y,rf_z,0,105,130);
		[rb_alfa,rb_beta,rb_gamma]=xyztoang(rb_x,rb_y,rb_z,0,105,130);

#		print("lf_alfa2",lf_alfa)	
#		print("lf_beta2",lf_beta)
#		print("lf_gamma2",lf_gamma)
	

		#%% stand by
		#position_pos = fix(position_angle/pi*180/135*1000);
		const = 1000/3.1416*180/135;
		position_angle =        [-rf_beta  * const , -rf_alfa * const , rf_gamma * const ,
				   	  lf_beta  * const ,  lf_alfa * const , lf_gamma * const ,
					 -rb_beta  * const , -rb_alfa * const ,-rb_gamma * const ,
					  lb_beta  * const ,  lb_alfa * const ,-lb_gamma * const ];

#		print("position_angle", position_angle)
		position_pos = np.trunc(position_angle);
		pose_print = position_pos - position_zero;
#		print("foot_step_x",foot_step_x)
		print("pose",j,200/time_step*j)
		print(pose_print[0], pose_print[1], pose_print[2])
		print(pose_print[3], pose_print[4], pose_print[5])
		print(pose_print[6], pose_print[7], pose_print[8])
		print(pose_print[9], pose_print[10], pose_print[11])

		for i in range(12): #=1:12
			#write(s,['#',num2str(i),'P' num2str(position_zero(i)+position_pos(i)) 'T',num2str(t),'!'],"uint8");
#			print("i",i)
			position  = np.int(position_zero[i] + position_pos[i]);
			send_text = "#"+str(i)+"P"+str(position)+"T"+str(t)+"!";
			result=ser.write(send_text.encode("UTF-8"))
#			print("send_text",send_text)
			#pause(0.05);
		time.sleep(0.1)
	return position_pos



################################################################
## main ##
################################################################

#portx = "/dev/ttyPS0"
portx = "/dev/ttyUSB2"
      
bps = 115200
timex = 5
    
ser=serial.Serial(portx,bps,timeout=timex)
    
# result=ser.write("我是东小东".encode("gbk"))
# print("写总字节数:",result)
    
## below is usrt sent moter poositions:

print("开始输出位置:")

position  = 500;

send_text = "#255P"+str(position)+"T100!";

result=ser.write(send_text.encode("gbk"))
print(send_text)

#result=ser.write("#1PRAD!");
#result=ser.write("#1PVER!");

## below send a sin position
ii=1;
x=0;

#while ii < 999 :
#	time.sleep(0.1)
#	x=x+0.2;
#
#	position  = np.uint(800*math.sin(x)+1500);
#
#	send_text = "#255P"+str(position)+"T100!";
#	#send_text = "#255P"+str(position)+"T100\r\n";
#
#	result=ser.write(send_text.encode("UTF-8"))
#	print(send_text)
#
#	ii=ii+1;

LEG_LENGTH = 0.282
MAX_FOOT_DISPLACEMENT = 0.125
control_points_x, control_points_y = scale_control_points(LEG_LENGTH, MAX_FOOT_DISPLACEMENT)
#print("CONTROL POINTS X: {}" .format(control_points_x))
#print("CONTROL POINTS Y: {}" .format(control_points_y))

# read walk step
#[foot_step_x, foot_step_y] =  trajectory_planner();
foot_step_y = [279.784161172141,279.186908630917,278.277002300741,277.115005759996,275.754049819188,274.240538017160,272.614797512661,270.911678712097,269.161106838920,267.388588516087,265.615676300516,263.860393977703,262.137625295807,260.459468691776,258.835560437576,257.273368512539,255.778459388321,254.354739796168,253.004675432211,251.729488445440,250.529335445055,249.403467658971,248.350374773650,247.367913887052,246.453424911499,245.603833671643,244.815743854573,244.085518884410,243.409354712554,242.783344437067,242.203535590490,241.665980864702,241.166782974233,240.702134295633,240.268351860143,239.861908219853,239.479458653788,239.117865129785,238.774217390597,238.445851488256,238.130366049206,237.825636514050,237.529827559732,237.241403878501,236.959139456928,236.682125469386,236.409776873649,236.141837771345,235.878385572837,235.619833984375,235.366934815005,235.120778581331,234.882793869785,234.654745398101,234.438730700184,234.237175341054,234.052826550887,233.888745149043,233.748295610053,233.635134103552,233.553194318798,233.506670861300,233.499999983990,233.537837387820,233.625032796419,233.766600976033,233.967688835097,234.233538197038,234.569443794824,234.980705986075,235.472577632684,236.050204528515,236.718558692392,237.482363770785,238.346011714935,239.313469810093,240.388177039675,241.572928663905,242.869747780455,244.279742513152,245.802947343574,247.438146958587,249.182680834279,251.032226612511,252.980560150118,255.019289931825,257.137563335859,259.321742025226,261.555043507194,263.817145658071,266.083750749154,268.326105232242,270.510471248619,272.597545513279,274.541820895751,276.290885669441,277.784655032326,278.954529112336,279.722471260182,280,280.157035805566,280.313935001052,280.470524379144,280.626649404310,280.782155999279,280.936890697097,281.090700792585,281.243434493039,281.394941068032,281.545070998166,281.693676122632,281.840609785429,281.985726980093,282.128884492803,282.269941043716,282.408757426397,282.545196645194,282.679124050441,282.810407471340,282.938917346399,283.064526851294,283.187112024028,283.306551887270,283.422728567744,283.535527412556,283.644837102345,283.750549761139,283.852561062818,283.950770334074,284.045080653759,284.135398948539,284.221636084743,284.303706956333,284.381530568888,284.455030119539,284.524133072763,284.588771231970,284.648880806803,284.704402476090,284.755281446392,284.801467506071,284.842915074851,284.879583248794,284.911435840669,284.938441415667,284.960573322422,284.977809719312,284.990133596013,284.997532790291,285.000000000000,284.997532790291,284.990133596013,284.977809719312,284.960573322422,284.938441415667,284.911435840669,284.879583248794,284.842915074851,284.801467506071,284.755281446392,284.704402476090,284.648880806803,284.588771231970,284.524133072763,284.455030119539,284.381530568888,284.303706956333,284.221636084743,284.135398948539,284.045080653759,283.950770334074,283.852561062818,283.750549761139,283.644837102345,283.535527412556,283.422728567744,283.306551887270,283.187112024028,283.064526851294,282.938917346399,282.810407471340,282.679124050441,282.545196645194,282.408757426397,282.269941043716,282.128884492803,281.985726980093,281.840609785429,281.693676122632,281.545070998166,281.394941068032,281.243434493039,281.090700792585,280.936890697097,280.782155999279,280.626649404310,280.470524379144,280.313935001052,280.157035805566,279.999981633974];
foot_step_x = [64.7962241881123,66.9281948440417,68.9047458853030,70.7336632089232,72.4214871275340,73.9734081466605,75.3932398834638,76.6834545548556,77.8452679930586,78.8787625824203,79.7830378556315,80.5563797434742,81.1964406427908,81.7004235554845,82.0652645599333,82.2878088081130,82.3649760998204,82.2939128724922,82.0721281639968,81.6976117592065,81.1689333218330,80.4853218436418,79.6467252163808,78.6538501502127,77.5081830286980,76.2119926070127,74.7683157296147,73.1809274684994,71.4542972659614,69.5935328088481,67.6043134670304,65.4928151996159,63.2656288705983,60.9296739234989,58.4921093443607,55.9602437964515,53.3414467404231,50.6430622626238,47.8723272239227,45.0362952138752,42.1417676524195,39.1952332255899,36.2028166749700,33.1702377847724,30.1027812274658,27.0052777406881,23.8820969166770,20.7371516924551,17.5739144363544,14.3954443359375,11.2044256057212,8.00321585206032,4.79390375878943,1.57837509240965,-1.64161412862989,-4.86435859807750,-8.08812024531625,-11.3110418570447,-14.5310594324856,-17.7458149855232,-20.9525715656347,-24.1481323068966,-27.3287653293420,-30.4901363081892,-33.6272504926440,-36.7344058958352,-39.8051592897264,-42.8323065223615,-45.8078785283573,-48.7231542260251,-51.5686912847656,-54.3343755033598,-57.0094892624317,-59.5827992016640,-62.0426629233360,-64.3771541374586,-66.5742052392998,-68.6217658465369,-70.5079753197783,-72.2213467459584,-73.7509592783283,-75.0866550986851,-76.2192365963852,-77.1406586438718,-77.8442100892641,-78.3246777823629,-78.5784856006467,-78.6038000458836,-78.4005930393470,-77.9706515537933,-77.3175226828730,-76.4463816630621,-75.3638092291274,-74.0774635011926,-72.5956303693243,-70.9266350598978,-69.0780962365581,-67.0560026071199,-64.8635905760431,-62.5000000000000,-61.2500000000000,-60,-58.7500000000000,-57.5000000000000,-56.2500000000000,-55,-53.7500000000000,-52.5000000000000,-51.2500000000000,-50,-48.7500000000000,-47.5000000000000,-46.2500000000000,-45,-43.7500000000000,-42.5000000000000,-41.2500000000000,-40,-38.7500000000000,-37.5000000000000,-36.2500000000000,-35,-33.7500000000000,-32.5000000000000,-31.2500000000000,-30,-28.7500000000000,-27.5000000000000,-26.2500000000000,-25,-23.7500000000000,-22.5000000000000,-21.2500000000000,-20.0000000000000,-18.7500000000000,-17.5000000000000,-16.2500000000000,-15,-13.7500000000000,-12.5000000000000,-11.2500000000000,-10.0000000000000,-8.75000000000000,-7.50000000000000,-6.25000000000000,-5.00000000000000,-3.75000000000000,-2.50000000000000,-1.25000000000000,0,1.25000000000000,2.50000000000000,3.75000000000000,5.00000000000000,6.25000000000001,7.50000000000001,8.75000000000000,10.0000000000000,11.2500000000000,12.5000000000000,13.7500000000000,15,16.2500000000000,17.5000000000000,18.7500000000000,20.0000000000000,21.2500000000000,22.5000000000000,23.7500000000000,25.0000000000000,26.2500000000000,27.5000000000000,28.7500000000000,30,31.2500000000000,32.5000000000000,33.7500000000000,35,36.2500000000000,37.5000000000000,38.7500000000000,40.0000000000000,41.2500000000000,42.5000000000000,43.7500000000000,45,46.2500000000000,47.5000000000000,48.7500000000000,50,51.2500000000000,52.5000000000000,53.7500000000000,55.0000000000000,56.2500000000000,57.5000000000000,58.7500000000000,60,61.2500000000000,62.5000000000000];

print("foot_step_x",foot_step_x)
while (1):
	serial_walk_control(0.0,0.0,0,10,0,-150,10,foot_step_x,foot_step_y);
	#serial_pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z,t)
	time.sleep(0.1)
	
ser.close()
    






















