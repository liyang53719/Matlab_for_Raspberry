%% connect raspberry
rpi = raspi('192.168.31.50', 'pi', 'raspberry')

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
old_modle = readRegister(i2cpwm, 0);
if (bitand(old_modle,16) == 0)
    write(i2cpwm, [0 17]);
end
write(i2cpwm, [PCA.PRESCALE 120])
write(i2cpwm, [0 old_modle]);

%% set output enable
write(i2cpwm, [0 1])

[a,b]=setPWM(i2cpwm, 0,1,150)

