% sim('ColorBasedTracking.mdl');
figure(1)
quiver(X(:,2),Y(:,2),V(:,2),U(:,2));
figure(2)
plot(V);
hold on
plot(U);