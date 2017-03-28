function dxdt = transitional_EOM_pitch_only(t,x)

global kf
m = 2.772;
g = 9.81;
W = -m * g;
Ft = -W;
Ixx = 0.0211;
Lt = 3.5 / 100;
Lx = 13.5/ 100;
Ly = 3.5 / 100;
Lf = 20  / 100;
beta = beta_t(t);
Cl_0 = 0.1974;
Cd_0 =  0.0789;

alpha = atan2(-x(1),x(2)) + x(3);


V = sqrt(x(1)^2 + x(2)^2);

if alpha > deg2rad(15)
    alpha_eff = deg2rad(15);
elseif alpha < deg2rad(-15)
    alpha_eff = -deg2rad(15);
else
    alpha_eff = alpha;
end

L = 1.225/2*V^2*(Cl_0+ alpha_eff*2*pi);

D = -1.225/2*V^2*(Cd_0+ 0.1*abs(alpha_eff)*2*pi);

Ff = kf*(rad2deg(x(3)) + 2*x(4)) .*  (sin(beta).*Ft * 0.2 + abs(L) .*0.15 );

dx1dt = (cos(beta-x(3))*Ft + L*cos(x(3)) + D * sin(x(3)) + W + Ff*cos(x(3)))/m;  %Vvertical
dx2dt = (sin(beta-x(3))*Ft + D*cos(x(3)) - L * sin(x(3)) + Ff * sin(x(3)))/m;           %Vhorzitontal
dx3dt = x(4);
dx4dt = ( -Ft*Lt*sin(beta-x(3)) - Ff*Lf - (-L*Lx + D*Ly))  /Ixx - x(4);
dxdt = [dx1dt dx2dt dx3dt dx4dt]';
