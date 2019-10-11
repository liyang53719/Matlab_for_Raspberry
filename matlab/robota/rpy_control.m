function pos_data = rpy_control(R, P, Y)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% def rpy_control(R, P, Y):

%     # Define the structure paramesters of robot
    l = 140 ;%# The length of platform.
    w = 140 ;%# The width of platform.    

%     # Rebuild the Euler-angle.
    R = R;
    P = P;
    Y = Y;   

%     # Foot position vector
    det = ([[ l/2, -0.55, -0.55]
            [ l/2, -0.55,  0.55]
            [-l/2, -0.55, -0.55]
            [-l/2, -0.55,  0.55],]);    

    pos_data = zeros(4, 3);
    for i =1:4
        detx = det(i, 1);
        dety = det(i, 2);
        detz = det(i, 3);

        rotx = ([[ 1,       0,       0     ]
                 [ 0,       cos(R), -sin(R)]
                 [ 0,       sin(R),  cos(R)]]);                      

        rotz = ([[ cos(P), -sin(P),  0     ]
                 [ sin(P),  cos(P),  0     ]
                 [ 0,       0,       1     ]]);                       

        roty = ([[ cos(Y),  0,      -sin(Y)]
                 [ 0,       1,       0     ]
                 [ sin(Y),  0,       cos(Y)]]);

        det_vec = ([[detx], [dety], [detz]]);

        off_vec = ([[ l/2, l/2, -l/2, -l/2] 
                    [ 0,   0,    0,    0  ] 
                    [-w/2, w/2, -w/2, w/2 ]]);

        pos_vec = rotx * roty * rotz * det_vec' - off_vec(:, i);%???

%         # Converte the global coordinate into the kinematics coordinates of each leg. 
        x = -pos_vec(2);
        z = pos_vec(1);
        if (mod(i, 2)==0)%(i%2)==0 :
            y = -pos_vec(3);
        else
            y = pos_vec(3);
        end

        pos_data(i, :) = ([x, y, z]);
    end

%     return pos_data
end

