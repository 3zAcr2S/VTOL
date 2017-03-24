tr = 0.6;

l = 5/0.12;
 x = 0:0.01:1;
    y = l ./ (1 - tr*x);

plot(x,y)
axis([0 1 0 150])
grid on
grid minor