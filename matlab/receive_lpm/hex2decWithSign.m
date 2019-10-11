function decval=hex2decWithSign(hexval, length)

decval = hex2dec(hexval);
sign = bitget(decval, 4*length);
negative_numbers = (sign == 1);
decval(negative_numbers) = decval(negative_numbers) - bitshift(1, 4*length);