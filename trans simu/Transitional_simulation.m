% Bi-Copter VTOL Transtional simulation
% Created by Haonan Zhang && Alibek Yertay @ PURDUE AAE
%%
clc
clear all
close all

TSPAN = [0 60];
Y0 = [0.0 0 0 0];
global kf
kf = 0.8;
[TOUT,XOUT] = ode45(@transitional_EOM_pitch_only,TSPAN,Y0);

figure(1)
plot(TOUT,XOUT(:,1));
hold on
plot(TOUT,XOUT(:,2));
legend('V_v','V_h')
figure(2)
hold on
plot(TOUT,XOUT(:,3));
alpha = atan2(-XOUT(:,1),XOUT(:,2)) + XOUT(:,3);
plot(TOUT,alpha);
legend('\theta','\alpha');





m = 2.772;
g = 9.81;
W = -m * g;
Ft = -W;
Ixx = 0.0211;
Lt = 3.5 / 100;
Lx = 13.5/ 100;
Ly = 3.5 / 100;
Lf = 20  / 100;
beta = beta_t(TOUT)';
Cl_0 = 0.1974;
Cd_0 =  0.0789;

V =sqrt(XOUT(:,1).^2 + XOUT(:,2).^2);

for i = [1:1:length(alpha)]
    
    if alpha(i) > deg2rad(15)
        alpha_eff(i) = deg2rad(15);
    elseif alpha(i) < deg2rad(-15)
        alpha_eff(i) = -deg2rad(15);
    else
        alpha_eff(i) = alpha(i);
    end
end
alpha_eff = alpha_eff';

L = 1.225/2.*V.^2.*(Cl_0+ alpha_eff.*2*pi);

D = -1.225/2.*V.^2.*(Cd_0+ 0.1*abs(alpha_eff).*2*pi);

Ff = kf*rad2deg(XOUT(:,3)+2*XOUT(:,4)) .*  (sin(beta).*Ft * 0.2 + L .*0.2 );





Mx_1 = -Ft.*Lt.*sin(beta-XOUT(:,3));
Mx_2 = -Ff.*Lf;
Mx_3 = +L.*Lx;
Mx_4 = -D.*Ly;
Mx_sum = Mx_1 + Mx_2+Mx_3+Mx_4;
figure(3)
plot(TOUT,Mx_1,TOUT,Mx_2,TOUT,Mx_3,TOUT,Mx_4,TOUT,Mx_sum)
legend('Mx1','Mx2','Mx3','Mx4','SUM');

figure(4)
plot(TOUT,L,TOUT,D,TOUT,Ff,TOUT,Ft*sin(beta-XOUT(:,3)),TOUT,Ft*cos(beta-XOUT(:,3)))
legend('L','D','Ff','Fh','Fv');
