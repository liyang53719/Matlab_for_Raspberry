%% connect raspberry
% rpi = raspi('192.168.31.50', 'pi', 'raspberry')
rpi = raspi('192.168.2.143','pi','raspberry')

%% scan i2c device
scanI2CBus(rpi,'i2c-1')
i2cpwm = i2cdev(rpi,'i2c-1','0x40')
%% same regs
PCA.SUBADR1            = 02;
PCA.SUBADR2            = 03;
PCA.SUBADR3            = 04;
PCA.MODE1              = 00;
PCA.PRESCALE           = 254;%FE;
PCA.LED0_ON_L          = 06;
PCA.LED0_ON_H          = 07;
PCA.LED0_OFF_L         = 08;
PCA.LED0_OFF_H         = 09;
PCA.ALLLED_ON_L        = 250;%'0xFA';
PCA.ALLLED_ON_H        = 251;%'0xFB';
PCA.ALLLED_OFF_L       = 252;%'0xFC';
PCA.ALLLED_OFF_H       = 253;%'0xFD';

%% set PWM frequency 
% prescale_value = round(osc_clock/(4096 * update_rate))-1
if(readRegister(i2cpwm, 254) ~= 120)
    old_modle = readRegister(i2cpwm, 0);
    if (bitand(old_modle,16) == 0)
        write(i2cpwm, [0 17]);
    end
    write(i2cpwm, [PCA.PRESCALE 120])
    write(i2cpwm, [0 old_modle]);
end

%% set output enable
write(i2cpwm, [0 1])

%% set pwm
% 0.5ms:103
% 1ms:205
% 2ms:410
% 2.5ms:510
[a,b]=setPWM(i2cpwm, 0,0,510)%%

x=0
while(1)
%     pause(0.01)
    x=x+0.01;
    setPWM(i2cpwm, 3,0,uint16(195*sin(x)+300));
    setPWM(i2cpwm, 0,0,uint16(195*sin(x)+300));
end