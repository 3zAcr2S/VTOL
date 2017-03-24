% close all
clear
clc
close all
Kpy = 5;
Kpx = 0.1;
Kpa = 1;
sim('ColorBasedTracking.slx');
figure(1)
quiver(X(:,2),Y(:,2),V(:,2),U(:,2));
hold on 
plot(Xr(1,2),Yr(1,2),'rx');
quiver(X(:,2),Y(:,2),sin(Theta(:,2)),cos(Theta(:,2)));

% figure(2)
% plot(V(:,1),V(:,2));
% hold on
% plot(U(:,1),U(:,2));
% hold off
% legend('V','U')
