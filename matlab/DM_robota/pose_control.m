function [AB] = pose_control(rpy_r,rpy_p,rpy_y,pos_x, pos_y, pos_z)
%UNTITLED3 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
%   # ����ṹ����
  b = 120;
  w = 120;
  l = 250;
%   # ����Ŀ��λ��
%   pos = np.mat([0.0,  0.0,  0.3 ]).T # Ŀ��λ������
%   rpy = np.array([0.0,  0.0,  0.0]) * math.pi / 180 # ŷ���ǣ���Ϊ����ֵ
%   # ��ŷ����ת��Ϊ��ת����
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
%   # �ṹ����
    body_struc = ([[ l / 2,  b / 2,  0]
                   [ l / 2, -b / 2,  0]
                   [-l / 2,  b / 2,  0]
                   [-l / 2, -b / 2,  0]])';
    footpoint_struc = ([[ l / 2,  w / 2,  0]
                        [ l / 2, -w / 2,  0]
                        [-l / 2,  w / 2,  0]
                        [-l / 2, -w / 2,  0]])';
%   # ���㵥��ĩ��λ������AB
  AB = zeros(3, 4);
  for i = 1:4
    AB(:, i) = - pos - rot_mat * body_struc(:, i) + footpoint_struc(:, i);
  end
%   print(AB)
end

