% serialportlist
clear

s = serialport("COM7",115200)

while(1)
    data = read(s,1,"uint8");
    if ( data~=hex2dec("3A") )
        continue;
    else
        if ( read(s,1,"uint8")==hex2dec("01") )
            if ( read(s,1,"uint8")==hex2dec("00") )
                if ( read(s,1,"uint8")==hex2dec("09") )
                    if ( read(s,1,"uint8")==hex2dec("00") )
                        cntL = read(s,1,"uint8");
                        cntH = read(s,1,"uint8");
                        cnt = cntH*255 + cntL;
                        sensor_data = read(s,cnt,"uint8");
                        
                        sensor.timer    = sensor_data(4)*16777215 + sensor_data(3)*56535 + sensor_data(2)*255 + sensor_data(1);
                        sensor.euler_x    = hex2decWithSign([dec2hex(sensor_data(6)),dec2hex(sensor_data(5))],4)/(pi*10000)*180;
                        sensor.euler_y    = hex2decWithSign([dec2hex(sensor_data(8)),dec2hex(sensor_data(7))],4)/(pi*10000)*180;
                        sensor.euler_z    = hex2decWithSign([dec2hex(sensor_data(10)),dec2hex(sensor_data(9))],4)/(pi*10000)*180
                        
%                         sensor.euler_x    = hex2decWithSign([dec2hex(sensor_data(8)),dec2hex(sensor_data(7)),dec2hex(sensor_data(6)),dec2hex(sensor_data(5))],8);
%                         sensor.euler_y    = hex2decWithSign([dec2hex(sensor_data(12)),dec2hex(sensor_data(11)),dec2hex(sensor_data(10)),dec2hex(sensor_data(9))],8);                        
%                         sensor.euler_z    = hex2decWithSign([dec2hex(sensor_data(16)),dec2hex(sensor_data(15)),dec2hex(sensor_data(14)),dec2hex(sensor_data(13))],8);

%                         scatter(sensor.timer,sensor.euler_x,'r'); hold on;
%                         scatter(sensor.timer,sensor.euler_y,'g'); hold on;
%                         scatter(sensor.timer,sensor.euler_z,'b'); hold on;
%                         drawnow
                    end
                end
            end
        end
    end    
end


% 80
% sensor.timer    = sensor_data(4)*16777215 + sensor_data(3)*56535 + sensor_data(2)*255 + sensor_data(1);
% sensor.gyu_x    = sensor_data(8)*16777215 + sensor_data(7)*56535 + sensor_data(6)*255 + sensor_data(5);
% sensor.gyu_y    = sensor_data(12)*16777215 + sensor_data(11)*56535 + sensor_data(10)*255 + sensor_data(9);
% sensor.gyu_z    = sensor_data(16)*16777215 + sensor_data(15)*56535 + sensor_data(14)*255 + sensor_data(13);
% sensor.acc_x    = sensor_data(20)*16777215 + sensor_data(19)*56535 + sensor_data(18)*255 + sensor_data(17);
% sensor.acc_y    = sensor_data(24)*16777215 + sensor_data(23)*56535 + sensor_data(22)*255 + sensor_data(21);
% sensor.acc_z    = sensor_data(28)*16777215 + sensor_data(27)*56535 + sensor_data(26)*255 + sensor_data(25);
% sensor.mag_x    = sensor_data(32)*16777215 + sensor_data(31)*56535 + sensor_data(30)*255 + sensor_data(29);
% sensor.mag_y    = sensor_data(36)*16777215 + sensor_data(35)*56535 + sensor_data(34)*255 + sensor_data(33);
% sensor.mag_z    = sensor_data(40)*16777215 + sensor_data(39)*56535 + sensor_data(38)*255 + sensor_data(37);
% sensor.qua_q0   = sensor_data(44)*16777215 + sensor_data(43)*56535 + sensor_data(42)*255 + sensor_data(41);
% sensor.qua_q1   = sensor_data(48)*16777215 + sensor_data(47)*56535 + sensor_data(46)*255 + sensor_data(45);
% sensor.qua_q2   = sensor_data(52)*16777215 + sensor_data(51)*56535 + sensor_data(50)*255 + sensor_data(49);
% sensor.qua_q3   = sensor_data(56)*16777215 + sensor_data(55)*56535 + sensor_data(54)*255 + sensor_data(53);
% sensor.euler_x  = sensor_data(60)*16777215 + sensor_data(59)*56535 + sensor_data(58)*255 + sensor_data(57);
% sensor.euler_y  = sensor_data(64)*16777215 + sensor_data(63)*56535 + sensor_data(62)*255 + sensor_data(61);
% sensor.euler_z  = sensor_data(68)*16777215 + sensor_data(67)*56535 + sensor_data(66)*255 + sensor_data(65);
% sensor.lacc_x   = sensor_data(72)*16777215 + sensor_data(71)*56535 + sensor_data(70)*255 + sensor_data(69);
% sensor.lacc_y   = sensor_data(76)*16777215 + sensor_data(75)*56535 + sensor_data(74)*255 + sensor_data(73);
% sensor.lacc_z   = sensor_data(80)*16777215 + sensor_data(79)*56535 + sensor_data(78)*255 + sensor_data(77)