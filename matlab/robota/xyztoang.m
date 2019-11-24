function [alfa,beta,gamma] = xyztoang(x,y,z,h,hu,hl)
% Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
% def xyztoang(x,y,z,h,hu,hl):     
% #Some sqrt¡¯s can be optimized out     
dyz=sqrt(y^2+z^2)  ;   
lyz=sqrt(dyz^2-h^2)   ;  
gamma_yz=-atan(y/z)  ;   
gamma_h_offset=-atan(h/lyz)    ; 
gamma=gamma_yz-gamma_h_offset   ;  
% #     
lxzp=sqrt(lyz^2+x^2)     ;
n=(lxzp^2-hl^2-hu^2)/(2*hu)     ;
beta=-acos(n/hl)     ;
% #     
alfa_xzp=-atan(x/lyz) ;    
alfa_off=acos((hu+n)/lxzp)   ;  
alfa=alfa_xzp+alfa_off   ;  
% #     
% return [alfa,beta,gamma]
end

