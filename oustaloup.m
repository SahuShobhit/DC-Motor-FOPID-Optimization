
function H = oustaloup(alpha, wb, wh, N)
% Approximate s^alpha using Oustaloup's recursive filter

H = tf(wh^alpha);

for k = -N:N
    wz = wb * (wh/wb)^((k + N + (1-alpha)/2)/(2*N+1));
    wp = wb * (wh/wb)^((k + N + (1+alpha)/2)/(2*N+1));

    H = H * tf([1 wz], [1 wp]);
end
end
