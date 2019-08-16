%% connect raspberry
rpi = raspi('192.168.2.145','pi','raspberry')

%% find Uart pins
% showPins(rpi)

%% open and connect an UART port
use_miniUART = 1;
if(use_miniUART)
    myserialdevice = serialdev(rpi,'/dev/ttyS0',115200);
else
    myserialdevice = serialdev(rpi,'/dev/ttyAMA0',115200);
end

myserialdevice.Timeout = 1;
% write(myserialdevice,['abcd!' 10 13],'char');
% write(myserialdevice,['#255P1500T100' 10 13],'char');
position  = 500;
write(myserialdevice,['#255P' num2str(position) 'T100!']);

write(myserialdevice,'#1PRAD!');
write(myserialdevice,'#1PVER!');
piserialread(myserialdevice,20)
% l = 20;
% rx_byte = 0;
% rx_cmd = 0;
% for i = 1:l
%     rx_byte = read(myserialdevice,1);
%     if(int8(rx_byte) == 13)
%         rx_byte = read(myserialdevice,1);
%         break
%     end
%     try
%         rx_cmd(i)=rx_byte;
%     catch
%         break
%     end
% end
% char(rx_cmd)

%% run a sin wave
clear plot_x plot_y1 plot_y2 plot_y3 plot_yy
x=0
ii=1;
nowtime = clock;
starttime = nowtime(5)*100+nowtime(6)
while(1)
    tic
    pause(0.01)
    x=x+0.2;
    position  = uint16(800*sin(x)+1500);
    write(myserialdevice,['#1P' num2str(position) 'T10!']);
    write(myserialdevice,['#2P' num2str(position) 'T10!']);
    write(myserialdevice,['#3P' num2str(position) 'T10!']);
    
    write(myserialdevice,'#1PRAD!');
    pos1 = piserialread(myserialdevice,20);
    
    write(myserialdevice,'#2PRAD!');
    pos2 = piserialread(myserialdevice,20);
    
    write(myserialdevice,'#3PRAD!');
    pos3 = piserialread(myserialdevice,20);
    
    nowtime = clock;
    plot_x(ii) = nowtime(5)*100+nowtime(6) - starttime;
    plot_y1(ii) = str2num(pos1(6:9));
    plot_y2(ii) = str2num(pos2(6:9));
    plot_y3(ii) = str2num(pos3(6:9));
    plot_yy(ii)= position;
    ii=ii+1;
%     if(rem(ii,10) == 1)
        plot(plot_x,plot_yy,'b',plot_x,plot_y1,'r',plot_x,plot_y2,'r',plot_x,plot_y3,'r')
%     end
    toc
end




