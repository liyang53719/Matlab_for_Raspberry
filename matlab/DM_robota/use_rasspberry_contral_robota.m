%% connect raspberry
rpi = raspi('192.168.31.125','pi','raspberry')

%% find Uart pins
% showPins(rpi)

%% open and connect an UART port
use_miniUART = 1;
if(use_miniUART)
    myserialdevice = serialdev(rpi,'/dev/ttyS0',115200);
else
    myserialdevice = serialdev(rpi,'/dev/ttyAMA0',115200);
end

position  = uint16(1500);
write(myserialdevice,['#1P' num2str(position) 'T10!']);
write(myserialdevice,['#2P' num2str(position) 'T10!']);
write(myserialdevice,['#3P' num2str(position) 'T10!']);
write(myserialdevice,['#4P' num2str(position) 'T10!']);
write(myserialdevice,['#5P' num2str(position) 'T10!']);
write(myserialdevice,['#6P' num2str(position) 'T10!']);
write(myserialdevice,['#7P' num2str(position) 'T10!']);
write(myserialdevice,['#8P' num2str(position) 'T10!']);
write(myserialdevice,['#9P' num2str(position) 'T10!']);
write(myserialdevice,['#10P' num2str(position) 'T10!']);
write(myserialdevice,['#11P' num2str(position) 'T10!']);
write(myserialdevice,['#12P' num2str(position) 'T10!']);