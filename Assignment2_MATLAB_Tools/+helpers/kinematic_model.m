function [x0,y0,x1c,y1c,x2,y2,delta,gamma1,V2,x1_dot,y1_dot,psi1_dot,psi2_dot]= kinematic_model(V1,delta_in,x1,y1,psi1, psi2)

p.L1=0.28;                          %wheel base of truck
p.L1c=0.055;                        % distance rear truck axle to first kingpin (positive means behind the axle)
p.L2=0.56;
p.delta_max =  38*pi/180;           % maximum steering angle front wheel [rad]
p.gamma_max = 110*pi/180;           % maximum articulation angle [rad]

% % limit maximum steering angle within mechanical limits (hard stops)
delta    = max(-p.delta_max, min(p.delta_max,delta_in));  

% calculate the articulation angle (angle between tractor and semitrailer)
gamma1   = psi1 - psi2;

% front axle position
x0       = x1 + p.L1*cos(psi1);
y0       = y1 + p.L1*sin(psi1);

% 5th wheel tractor (coupling to semi-trailer) position
x1c      = x1 + p.L1c*cos(psi1);
y1c      = y1 + p.L1c*sin(psi1);

% semitrailer center axle position
x2       = x1c - p.L2*cos(psi2);
y2       = y1c - p.L2*sin(psi2);

% tractor dynamics
x1_dot   =  V1*cos(psi1);
y1_dot   =  V1*sin(psi1);
psi1_dot = (V1/p.L1)*tan(delta);

%semitrailer dynamics
V2       =   V1*cos(gamma1) - psi1_dot*p.L1c*sin(gamma1);
psi2_dot = ( V1*sin(gamma1) + psi1_dot*p.L1c*cos(gamma1) )/p.L2;

end
