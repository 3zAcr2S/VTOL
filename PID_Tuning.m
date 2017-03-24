clear all
close all
clc
inch = 0.0254;
foot = 0.3048;
pound = 4.44822;
g = 9.81;
Dia = 12*inch;
C_p_0 = 0.0448;
Rho = 0.00238;

%Parameter

syms db1 db2 db3 beta_1 beta_2 h_r h_l l_r l_l m g psi phi theta alpha beta gamma...
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

s = solve([F_i_var ==0,Tau_i_var(1)==0]);
theta_e = double(s.theta(1));
F_1_e = double(s.F_1(1));
F_2_e = double(s.F_2(1));
%%



% II = [141660537.14 28996.94 457962.8;28996.94 18896041.02 3929.66;457962.80 3929.66 159679095.70] / 10^9;
II = [141660537.14 0 457962.8;0 18896041.02 0;457962.80 0 159679095.70] / 10^9;
F13 = [p + tan(theta)*sin(phi)*q + tan(theta)*cos(phi)*r;
    cos(phi)*q - sin(phi)*r;
    (sin(phi)/cos(theta))*q + cos(phi)/cos(phi)*r];
F36 = II\[(II(2,2) - II(3,3)) * q * r + II(1,3)*p*q + Tau_i(1);
    (II(3,3)-II(1,1))*p*r + II(1,3)*(r.^2 + p.^2) + Tau_i(2);
    (II(1,1)-II(2,2))*p*q-II(1,3)*q*r+Tau_i(3)];
F_EOM_Nonlin = [F13; F36];
states = [phi, theta, psi, p, q, r];
ctrl_input = [F_1, F_2, beta_1, beta_2];
A = double(subs(jacobian(F_EOM_Nonlin, states), [states, ctrl_input, design_para], [0, theta_e, 0, 0, 0, 0,F_1_e,F_2_e,beta_1_e,beta_2_e,design_var]));
B = double(subs(jacobian(F_EOM_Nonlin, ctrl_input), [states, ctrl_input, design_para], [0, theta_e, 0, 0, 0, 0,F_1_e,F_2_e,beta_1_e,beta_2_e,design_var]));
Cont = ctrb(A, B);
No_Uncon_states = length(A) - rank(Cont);
C = eye(6);
Observ = obsv(A, C);
No_Unobs_states = length(A) - rank(Observ);
D = zeros(6,4);

sys_mimo = ss(A,B,C,D);
tran_func = tf(sys_mimo);
yp = 2; yi = 0; yd = 0;
Cs = tf([yd yp yi], [1 0]);
CsGs = Cs*tran_func;

syms x
for r_tf = [1:1:6]
    for c_tf = [1:1:4]
        E_R(r_tf, c_tf) =  1/(1 + CsGs(r_tf,c_tf));
        [num,den] = tfdata(E_R(r_tf, c_tf),'v');
         p = poly2sym(den);
         soln = solve([p == 0], [x]);
         n = length(soln);
         for i = [1:1:n]
             
            if (soln(i) < 0)
                disp('Stable')
            else
                disp('not stable')
            end
         end
    end
end



