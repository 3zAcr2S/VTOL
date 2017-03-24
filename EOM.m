clear all
close all
clc
inch = 0.0254;
foot = 0.3048;
pound = 4.44822;
g = 9.81;
Dia = 12*inch;
C_p_0 = 0.03;
Rho = 0.00238;

%Parameter

syms db1 db2 db3 beta_1 beta_2 h_r h_l l_r l_l m psi phi theta alpha beta gamma...
    n_1_RPS n_2_RPS rho C_p C_f Dia F_1 F_2 p q r real


db1_num = 2.69/1000;
db2_num = 0;
db3_num = -9.74/1000;
l_l_num = 13 * inch;
l_r_num = 13 * inch;
beta_1_num = atan(db1_num/-db3_num);
beta_2_num = atan(db1_num/-db3_num);


beta_1_e = beta_1_num;
beta_2_e = beta_2_num;

h_r_num = 0.02;
h_l_num = 0.02;
g_num = 9.81;
m_num = 2;
Dia_num = 12*inch;
C_p_num = 0.03;
C_f_num = 0.11;

% d_b =  % vector distance, from c.m. to rod Rcm_rod (in b^ frame)
% db1 = % distance from c.m. to rod (in -b^_1 direction)
% db2 = % distance from c.m. to rod (in  b^_2 direction)
% db3 = % distance from c.m. to rod (in  b^_3 direction)
% l_r =  13 * inch; % distance from c.m. to motor_1(X_1) (in b^_2 directom)
% l_l = 13 * inch; % distance from c.m.  to motor_1(X_1) (in -b^_2 directom)
% h_r =
% beta_1 = ; % angle between F^_1 and -b^_3)
% beta_2 = ; % angle between F^_2 and -b^_3)

R_OX_1 = [db1 + sin(beta_1) * h_r,  l_r, -cos(beta_1) * h_r + db3];
R_OX_2 = [db1 + sin(beta_2) * h_l, -l_l, -cos(beta_2) * h_l + db3];


R_OX_1 = [db1,  l_r, db3];
R_OX_2 = [db1, -l_l, db3];



W_i = [0 0 m*g]; % weight in inertial frame

yaw = psi;
roll = phi;
pitch = theta;
alpha = yaw;
beta  = pitch;
gamma   = roll;

i_C_b = [cos(alpha) * cos(beta),  cos(alpha)*sin(beta)*sin(gamma) - sin(alpha), cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma); ...
    sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)* sin(gamma); ...
    -sin(beta), cos(beta)*sin(gamma),cos(beta)*cos(gamma)];
W_b = W_i * i_C_b ;

% F_1 = % thrust from motor 1
% F_2 = % thrust from motor 2

Dir_F_1 = [sin(beta_1), 0 , -cos(beta_1)]; % direction of force in b^ frame
Dir_F_2 = [sin(beta_2), 0 , -cos(beta_2)]; % direction of force in b^ frame

F = [ F_1*sin(beta_1) + F_2*sin(beta_2) , 0 , -F_1*cos(beta_1) - F_2 * cos(beta_2)];
W = W_b;

% C_p = C_p_0;
% Tau_drag_1 = C_p * rho * (n_1_RPS)^2 * (Dia/12).^5 / (2*pi) * Dir_F_1;  % direction of torque in b^ frame
% Tau_drag_2 = C_p * rho * (n_2_RPS)^2 * (Dia/12).^5 / (2*pi) * Dir_F_2;

Tau_drag_1 = C_p / C_f * Dia / (2*pi)*F_1 * Dir_F_1;
Tau_drag_2 = -C_p / C_f * Dia / (2*pi)*F_2 * Dir_F_2;
% Tau =  cross(R_OX_1 + R_OX_2,F) + Tau_drag_1 + Tau_drag_2;
Tau = cross(R_OX_1,Dir_F_1*F_1)+cross(R_OX_2,Dir_F_2*F_2) + Tau_drag_1 + Tau_drag_2;



%% Hovering



Tau_i = i_C_b * Tau';

F_i = i_C_b *( F' + W');
design_para = [db1, db2, db3,l_l,l_r,beta_1,beta_2,h_r,h_l,g,m,Dia,C_p,C_f];
design_var = [db1_num, db2_num, db3_num,l_l_num,l_r_num,beta_1_num,beta_2_num,h_r_num,h_l_num,g_num,m_num,Dia_num,C_p_num,C_f_num];
Tau_i_var = subs(Tau_i, [design_para,psi,phi], [design_var,0,0]);
F_i_var = subs(F_i, [design_para,psi,phi], [design_var,0,0]);

s = solve(F_i_var == 0,Tau_i_var(1) == 0, theta, F_1, F_2);
theta_e = double(s.theta(1));
F_1_e = double(s.F_1(1));
F_2_e = double(s.F_2(1));
%%

theta_e = beta_1_num;
F_1_e = m_num/2*g;
F_2_e = m_num/2*g;




% II = [141660537.14 28996.94 457962.8;28996.94 18896041.02 3929.66;457962.80 3929.66 159679095.70] / 10^9;
II = [141660537.14 0 457962.8;0 18896041.02 0;457962.80 0 159679095.70] / 10^9;
F13 = [p + tan(theta)*sin(phi)*q + tan(theta)*cos(phi)*r;
    cos(phi)*q - sin(phi)*r;
    (sin(phi)/cos(theta))*q + cos(phi)/cos(phi)*r];
F46 = II\[(II(2,2) - II(3,3)) * q * r + II(1,3)*p*q + Tau_i(1);
    (II(3,3)-II(1,1))*p*r + II(1,3)*(r.^2 + p.^2) + Tau_i(2);
    (II(1,1)-II(2,2))*p*q-II(1,3)*q*r+Tau_i(3)];
F_EOM_Nonlin = [F13; F46];   % EOM of states
sim_EOM = subs(F_EOM_Nonlin,[design_para,pi,g],[design_var,3.1415926,9.81]);  % simplified EOM relacing most parameters
states = [phi, theta, psi, p, q, r];
ctrl_input = [F_1, F_2, beta_1, beta_2];
A = double(subs(jacobian(F_EOM_Nonlin, states), [states, ctrl_input, design_para,pi,g], [0, theta_e, 0, 0, 0, 0,F_1_e,F_2_e,beta_1_e,beta_2_e,design_var,3.1415926,9.81]));
Ai = double([A,zeros(6,3);eye(3),zeros(3,6)]);
B = double(subs(jacobian(F_EOM_Nonlin, ctrl_input), [states, ctrl_input, design_para,pi,g], [0, theta_e, 0, 0, 0, 0,F_1_e,F_2_e,beta_1_e,beta_2_e,design_var,3.1415926,9.81]));
Bi = double([B;zeros(3,4)]);
W = [zeros(6,3);eye(3)];


% Cont = ctrb(Ai, Bi);
% No_Uncon_states = length(Ai) - rank(Cont);
C = eye(6);
% Observ = obsv(Ai, C);
% No_Unobs_states = length(Ai) - rank(Observ);
D = zeros(6,4);
Q = 20.*eye(9);
R = 5.*eye(4);
K = lqr(Ai, Bi, Q, R);



%% Thrust Simulation
close all

rpm_data = load('RPM.txt');
throttle_25 = rpm_data(:,1);
rps_25 = rpm_data(:,2)/6;
throttle_22_7 = rpm_data(:,3);
rps_22_7 = rpm_data(:,4)/6;
order_num = 2;
poly_coeff_25 = polyfit(throttle_25,rps_25,order_num);
poly_coeff_22_7 = polyfit(throttle_22_7,rps_22_7,order_num);

throttle_range = [20:1:165];  % Throttle % Thrust Setting For Simulation
thrust_range = [0:0.1:13];

figure(1)
hold on
plot(throttle_range,polyval(poly_coeff_25,throttle_range),'r','LineWidth',1.5);
plot(throttle_25,rps_25,'r--','LineWidth',1.5);
plot(throttle_range,polyval(poly_coeff_22_7,throttle_range),'b','LineWidth',1.5);
plot(throttle_22_7,rps_22_7,'b--','LineWidth',1.5);
legend('25fit','25','22.7fit','22.7')




thrust_25   = C_f_num * Rho * rps_25.^2 * (Dia_num/foot)^4 * pound;
thrust_22_7 = C_f_num * Rho * rps_22_7.^2 * (Dia_num/foot)^4 * pound;
thrust_coeff_25   = polyfit(throttle_25,thrust_25,order_num+1);
thrust_coeff_22_7 = polyfit(throttle_22_7,thrust_22_7,order_num+1);

throttle_coeff_25   = polyfit(thrust_25  ,throttle_25  ,order_num+2);
throttle_coeff_22_7 = polyfit(thrust_22_7,throttle_22_7,order_num+2);


figure(2)
hold on
plot(throttle_range,polyval(thrust_coeff_25,throttle_range),'r','LineWidth',1.5);
plot(throttle_25,thrust_25,'r--','LineWidth',1.5);
plot(throttle_range,polyval(thrust_coeff_22_7,throttle_range),'b','LineWidth',1.5);
plot(throttle_22_7,thrust_22_7,'b--','LineWidth',1.5);
xlabel('throttle')
ylabel('thrust [N]')
legend('25fit','25','22.7fit','22.7')

figure(3)
hold on
plot(thrust_range,polyval(throttle_coeff_25,thrust_range),'r','LineWidth',1.5);
plot(thrust_25,throttle_25,'r--','LineWidth',1.5);
plot(thrust_range,polyval(throttle_coeff_22_7,thrust_range),'b','LineWidth',1.5);
plot(thrust_22_7,throttle_22_7,'b--','LineWidth',1.5);
ylabel('throttle')
xlabel('thrust [N]')
legend('25fit','25','22.7fit','22.7')
grid minor


%% Beta / iNPUT Calibration



IMU_DATA_RAW = load('Sensor_test.txt');
IMU_DATA = IMU_DATA_RAW(:,[2,1,3,5,4,6,8,7,9]);
Mean_IMU = mean(IMU_DATA(:,4:9));
Mean_IMU = zeros(1,6);
Variance_IMU = var(IMU_DATA(:,4:9));
figure(8)
plot(IMU_DATA(:,1:6))
legend('roll','pitch','yaw','roll_k','pitch_k','yaw_k','roll_rate','pitch_rate','yaw_rate')



%%
close all
% Loading RAW DATA
Beta_angle_DATA_RAW = load('Angle.dat');

Beta_left = deg2rad(Beta_angle_DATA_RAW(:,2));
Beta_right = deg2rad(Beta_angle_DATA_RAW(:,4));

Servo_INPUT_left = Beta_angle_DATA_RAW(:,1);
Servo_INPUT_right = Beta_angle_DATA_RAW(:,3);

% Setting Linear Fit Order Number
order_num = 2;
input_range = [30:160];
Beta_range = [-0.6:0.02:1];

% Linear Fit Beta = F(servo(in))
beta_left_coeff = polyfit(Servo_INPUT_left,Beta_left,order_num);
beta_right_coeff = polyfit(Servo_INPUT_right,Beta_right,order_num);



figure(11)
hold on
plot(input_range,polyval(beta_left_coeff,input_range),'r','LineWidth',1.5);
plot(Servo_INPUT_left,Beta_left,'ro','LineWidth',1.5);
plot(input_range,polyval(beta_right_coeff,input_range),'b','LineWidth',1.5);
plot(Servo_INPUT_right,Beta_right,'bo','LineWidth',1.5);
legend('left_f_i_t','left','right_f_i_t','right')

% Linear Fit servo(in) = F(Beta)

servo_left_coeff = polyfit(Beta_left,Servo_INPUT_left,order_num);
servo_right_coeff = polyfit(Beta_right,Servo_INPUT_right,order_num);
figure(12)
hold on
plot(Beta_range,polyval(servo_left_coeff,Beta_range),'r','LineWidth',1.5);
plot(Beta_left,Servo_INPUT_left,'ro','LineWidth',1.5);
plot(Beta_range,polyval(servo_right_coeff,Beta_range),'b','LineWidth',1.5);
plot(Beta_right,Servo_INPUT_right,'bo','LineWidth',1.5);
legend('left_f_i_t','left','right_f_i_t','right')



%% Simulation
 sim('simulation')
 figure(10)
 plot(ScopeData2.time,ScopeData2.signals.values)
 legend('F_1','F_2','\beta_1','\beta_2')
 figure(9)
 plot(ScopeData.time,ScopeData.signals.values)
legend('roll','pitch','yaw','roll rate','pitch rate','yaw rate')

%%
syms kalAngleX kalAngleY kalAngleZ gyroX gyroY gyroZ real
ardustates = [kalAngleX kalAngleY kalAngleZ gyroX gyroY gyroZ];

GainK = -K*ardustates'

%%

beta_1_s = (ScopeData2.signals.values(:,4))';



n = length(beta_1_s);
beta_1_f(1) = beta_1_s(1);
for ct = 2:1:n
    beta_1_f(ct) = 0.2*beta_1_s(ct) + 0.8*beta_1_f(ct - 1);
end
close all
figure(100)
plot(beta_1_f,'r')
hold on
plot(beta_1_s,'b')

%%
