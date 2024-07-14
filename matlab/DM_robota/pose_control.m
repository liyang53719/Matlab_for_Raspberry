function [AB] = pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z)
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
%   # 定义结构参数
  b = 120;
  w = 120;
  l = 250;
%   # 定义目标位姿
%   pos = np.mat([0.0,  0.0,  0.3 ]).T # 目标位置向量
%   rpy = np.array([0.0,  0.0,  0.0]) * math.pi / 180 # 欧拉角，化为弧度值
%   # 将欧拉角转换为旋转矩阵
%   R, P, Y = rpy[0], rpy[1], rpy[2]
    R = rpy_r;
    P = rpy_p;
    Y = rpy_y;
    pos = [pos_x, pos_y, pos_z]';
    rotx = ([[ 1,       0,       0     ]
             [ 0,       cos(R), -sin(R)]
             [ 0,       sin(R),  cos(R)]]);
    roty = ([[ cos(P),  0,      -sin(P)]
             [ 0,       1,       0     ]
             [ sin(P),  0,       cos(P)]]);
    rotz = ([[ cos(Y), -sin(Y),  0     ]
             [ sin(Y),  cos(Y),  0     ]
             [ 0,       0,       1     ]]);
  rot_mat = rotx * roty * rotz;
%   # 结构参数
    body_struc = ([[ l / 2,  b / 2,  0]
                   [ l / 2, -b / 2,  0]
                   [-l / 2,  b / 2,  0]
                   [-l / 2, -b / 2,  0]])';
    footpoint_struc = ([[ l / 2,  w / 2,  0]
                        [ l / 2, -w / 2,  0]
                        [-l / 2,  w / 2,  0]
                        [-l / 2, -w / 2,  0]])';
%   # 计算单腿末端位置向量AB
  AB = zeros(3, 4);
  for i = 1:4
    AB(:, i) = - pos - rot_mat * body_struc(:, i) + footpoint_struc(:, i);
  end
%   print(AB)
end

