function beta = beta_t(t)

n = length(t);
for i = [1:1:n]
    
    if t(i) < 10
        beta(i) = 1/2*pi*t(i)/10;
    else
        beta(i) = 1/2*pi;
    end
end
% beta = deg2rad(90);