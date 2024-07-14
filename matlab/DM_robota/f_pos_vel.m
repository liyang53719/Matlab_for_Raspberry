%% 位置速度
function [can_id_s] = f_pos_vel(s, can_id, pos, vel)
can_id_h = fix(can_id/100);
can_id_l = mod(can_id, 100);

q = quantizer('single');
pos_hex = num2hex(q,pos);
vel_hex = num2hex(q,vel);

udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, hex2dec(pos_hex(7:8)), hex2dec(pos_hex(5:6)), hex2dec(pos_hex(3:4)), hex2dec(pos_hex(1:2)), hex2dec(vel_hex(7:8)), hex2dec(vel_hex(5:6)), hex2dec(vel_hex(3:4)), hex2dec(vel_hex(1:2))];
write(s,udata,"uint8")
end
