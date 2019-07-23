%% setPWM function
function [on_read, off_read] = setPWM(i2cpwm, channel, on, off)

PCA.LED0_ON_L          = 06;
PCA.LED0_ON_H          = 07;
PCA.LED0_OFF_L         = 08;
PCA.LED0_OFF_H         = 09;

write(i2cpwm, [PCA.LED0_ON_L+4*channel bitand(on, uint16(255))])
write(i2cpwm, [PCA.LED0_ON_H+4*channel bitshift(on,-8)])
write(i2cpwm, [PCA.LED0_OFF_L+4*channel bitand(off, uint16(255))])
write(i2cpwm, [PCA.LED0_OFF_H+4*channel bitshift(off,-8)])

on_read = uint16(readRegister(i2cpwm, PCA.LED0_ON_L+4*channel)) + bitshift(uint16(readRegister(i2cpwm, PCA.LED0_ON_H+4*channel)),8);
off_read = uint16(readRegister(i2cpwm, PCA.LED0_OFF_L+4*channel)) + bitshift(uint16(readRegister(i2cpwm, PCA.LED0_OFF_H+4*channel)),8);
end