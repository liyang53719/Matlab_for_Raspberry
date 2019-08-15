function [readout] = piserialread(myserialdevice,l)
%PISERIALREAD 此处显示有关此函数的摘要
%   此处显示详细说明
rx_byte = 0;
rx_cmd = 0;
for i = 1:l
    rx_byte = read(myserialdevice,1);
    if(int8(rx_byte) == 13)
        rx_byte = read(myserialdevice,1);
        break
    end
    try
        rx_cmd(i)=rx_byte;
    catch
        break
    end
end
readout = char(rx_cmd);
end

