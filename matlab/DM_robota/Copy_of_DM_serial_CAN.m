% serialportlist
% s = serialport("COM5",115200,"Timeout",0.05);

%       fix    fix   length     
% udata = [0xAA, 0, 0, 0x08, 0, 0, 1, 1, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
%% main below

% f_read_can_id(s)
f_moter_en_all(s)
% f_pos_vel(s, can_id, pos, vel)
% f_moter_en(s, '141')
% f_moter_en(s, '122')
% f_pos_vel(s, '140', 0, 0.5)
f_moter_go_zero_all(s)
pause(5)
f_moter_disen_all(s)

%% function below
%% 读取can id
function [can_id_s] = f_read_can_id(s)
udata = [0xAA, 0, 0, 0x08, 0, 0, 0x07, 0xff, 0x55, 0x00, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00];
write(s,udata,"uint8")
end

%% 使能电机
function [can_id_s] = f_moter_en(s, can_id)
% can_id_h = fix(can_id/100);
% can_id_l = mod(can_id, 100);
can_id_h = hex2dec(can_id(1));
can_id_l = hex2dec(can_id(2:3));


udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
write(s,udata,"uint8")
end

%% 使能全部电机
function [can_id_s] = f_moter_en_all(s)
f_moter_en(s, '110'); pause(0.1)
f_moter_en(s, '111'); pause(0.1)
f_moter_en(s, '112'); pause(0.1)
f_moter_en(s, '120'); pause(0.1)
f_moter_en(s, '121'); pause(0.1)
f_moter_en(s, '122'); pause(0.1)
f_moter_en(s, '130'); pause(0.1)
f_moter_en(s, '131'); pause(0.1)
f_moter_en(s, '132'); pause(0.1)
f_moter_en(s, '140'); pause(0.1) 
f_moter_en(s, '141'); pause(0.1) 
f_moter_en(s, '142'); pause(0.1) 
end

%% 失能电机
function [can_id_s] = f_moter_disen(s, can_id)
% can_id_h = hex2dec(fix(can_id/100));
% can_id_l = hex2dec(mod(can_id,100));
can_id_h = hex2dec(can_id(1));
can_id_l = hex2dec(can_id(2:3));

udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
write(s,udata,"uint8")
end

%% 失能全部电机
function [can_id_s] = f_moter_disen_all(s)
f_moter_disen(s, '110'); pause(0.2)
f_moter_disen(s, '111'); pause(0.2)
f_moter_disen(s, '112'); pause(0.2)
f_moter_disen(s, '120'); pause(0.2)
f_moter_disen(s, '121'); pause(0.2)
f_moter_disen(s, '122'); pause(0.2)
f_moter_disen(s, '130'); pause(0.2)
f_moter_disen(s, '131'); pause(0.2)
f_moter_disen(s, '132'); pause(0.2)
f_moter_disen(s, '140'); pause(0.2)
f_moter_disen(s, '141'); pause(0.2)
f_moter_disen(s, '142'); pause(0.2)
end

%% 保存零点
function [can_id_s] = f_set_zero_point(s, can_id)
% can_id_h = hex2dec(fix(can_id/100));
% can_id_l = hex2dec(mod(can_id,100));
can_id_h = hex2dec(can_id(1));
can_id_l = hex2dec(can_id(2:3));

udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
write(s,udata,"uint8")
end

%% 清除电机状态
function [can_id_s] = f_clear_moter(s, can_id)
% can_id_h = hex2dec(fix(can_id/100));
% can_id_l = hex2dec(mod(can_id,100));
can_id_h = hex2dec(can_id(1));
can_id_l = hex2dec(can_id(2:3));

udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB];
write(s,udata,"uint8")
end
%% 位置速度
function [can_id_s] = f_pos_vel(s, can_id, pos, vel)
% can_id_h = fix(can_id/100);
% can_id_l = mod(can_id, 100);
can_id_h = hex2dec(can_id(1));
can_id_l = hex2dec(can_id(2:3));

q = quantizer('single');
pos_hex = num2hex(q,pos);
vel_hex = num2hex(q,vel);

udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, hex2dec(pos_hex(7:8)), hex2dec(pos_hex(5:6)), hex2dec(pos_hex(3:4)), hex2dec(pos_hex(1:2)), hex2dec(vel_hex(7:8)), hex2dec(vel_hex(5:6)), hex2dec(vel_hex(3:4)), hex2dec(vel_hex(1:2))];
write(s,udata,"uint8")
end

%% 全部电机到零点
function [can_id_s] = f_moter_go_zero_all(s)
f_pos_vel(s, '110', 0, 0.5); pause(0.5)
f_pos_vel(s, '120', 0, 0.5); pause(0.5)
f_pos_vel(s, '130', 0, 0.5); pause(0.5)
f_pos_vel(s, '140', 0, 0.5); pause(0.5)
f_pos_vel(s, '111', 0, 0.5); pause(0.5)
f_pos_vel(s, '121', 0, 0.5); pause(0.5)
f_pos_vel(s, '131', 0, 0.5); pause(0.5)
f_pos_vel(s, '141', 0, 0.5); pause(0.5)
f_pos_vel(s, '112', 0, 0.5); pause(0.5)
f_pos_vel(s, '122', 0, 0.5); pause(0.5)
f_pos_vel(s, '132', 0, 0.5); pause(0.5)
f_pos_vel(s, '142', 0, 0.5); pause(0.5)

end

