% As described in the report the state space model can be represented as
%****STATE SPACE MODEL****%
m = 2325 ; %weight of the vechile in Kg
I = 4132 ; %vechile yaw moment of inertia(Kg.m^2)
a = 1.430 ; % and the distances of the front wheel axles from the centre of
% gravity is a(m)
b = 1.59515 ; %and the distances of the rear wheel axles from the centre of
% gravity is b(m)
Uc = 20 ; % Assume that vechile is driving at constant velocity(20m/s)  
Cr = 96000 ; %cornering stiffness of front tyre
Cf = 80000 ;%cornering stiffness of front tyre
A = [-(Cf+Cr)/(Uc*m)    -Uc-(a*Cf-b*Cr)/(Uc*m)  0  0;
    -(a*Cf-b*Cr)/(Uc*I) -(a*a*Cf-b*b*Cr)/(Uc*I) 0  0;
            0                      1            0  0;  
            1                      0           20 0];
B = [ Cf/m
     a*Cf/I
       0
       0];
C = [0 0 0 1];
D = 0;
s = tf('s');
states = {'v' 'r' '\theta' 'E'};
inputs = {'\delta_f(t)'};
output = {'E'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',output);
%We get the following result. 
% sys =
%  
%   A = 
%            x1      x2      x3      x4
%    x1  -3.785  -19.17       0       0
%    x2  0.4687  0.9763       0       0
%    x3       0       1       0       0
%    x4       1       0      20       0
%  
%   B = 
%           u1
%    x1  34.41
%    x2  27.69
%    x3      0
%    x4      0
%  
%   C = 
%        x1  x2  x3  x4
%    y1   0   0   0   1
%  
%   D = 
%        u1
%    y1   0
%  
% Continuous-time state-space model.
sys_tf = tf(sys_ss) 
%  we get the following 
% sys_tf =
%  
%   34.41 s^2 - 10.53 s + 2418
%   ---------------------------
%   s^4 + 2.809 s^3 + 5.289 s^2
figure;
step(sys_tf,5);
% %Graph is shown in the report.
% % we can clearly see from the the TF as well as the
% % Graph. that sys is unstable.

% So, we use velocity feedback with a gain of 10 
% % New TF will be :
sys_tf1 = (sys_tf*s)/((1+10*s)*sys_tf*s);
Gc = 1+1/(2*s)+10*s;
sys_tf2 = (Gc*sys_tf*s)/(1+(1+10*s)*Gc*sys_tf*s);
sys_tf3 = (sys_tf2/s)/(1+sys_tf2/s)
figure;
step(sys_tf3,80);
figure;
step(sys_tf3/s,80);
title('RAMP RESPONSE');
