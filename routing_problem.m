%% ROUTING
clear
clc
close all
n = 10*2+1;

map = zeros(n,n);

x = 1;
y = n;

map(x,y) = 1;
while ((x ~= n) || (y ~= 1));
    
    
    
    x = x + round(rand*2-0.7);
    
  
    if x > n
        x = n;
    end
    if x <1
        x = 1;
    end
        map(x,y) = 1;

      y = y - round(rand*2 - 0.8 );
    if y > n
        y = n;
    end
    if y < 1
        y = 1;
    end
        map(x,y) = 1;

    
   figure(1)
   hold on
   plot(x,y,'*-','MarkerSize',10,'LineWidth',3)

    
end
ct = 1;

for yy = n:-1:1
for    xx = 1:1:n


if map(yy,xx) == 1
    ploter(ct,:) = [xx,yy];
    ct =  ct + 1;
   
end
end
end
% figure(2)
%  plot(ploter(:,1),n - ploter(:,2),'*-','MarkerSize',10,'LineWidth',3)

grid on
