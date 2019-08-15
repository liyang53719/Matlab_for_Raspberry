function [readout] = piserialread(myserialdevice,l)
%PISERIALREAD �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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

