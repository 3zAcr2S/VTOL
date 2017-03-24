Cl = 0.2;


M = 4;

g = 9.81;
W = M*g;

v = 25;

rho = 1.2;

A = W / (0.5*rho*v^2*Cl)


chord_root = 20 * 0.0254;
b = 48* 0.0254;
tr = 0.6;
chord_end = tr*chord_root;

S = b*(chord_root*(1+tr)/2)