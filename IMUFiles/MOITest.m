clear;
clc;

% CubeSat Components
p = % angular velocity along i
q = % angular velocity along j
r = % angular velocity along k

p_dot = % angular acceleration along i
q_dot = % angular acceleration along j
r_dot = % angular acceleration along k

% Reaction Wheel Components


n = size(p);
for i = 1:n
    A(i) = [p_dot(i) q_dot(i)-(p(i)*r(i)) r_dot(i)+p(i)q(i) -q(i)r(i) (q(i)^2)-(r(i)^2) q(i)r(i);
            ]
    
    A_pseudo(i) = ((A'(i)A(i))^(-1))*A'(i)
    x(i) = A_pseudo(i)*T(i);
end


x(i)
% plots
plot(x())% plot I_11 vs kth step
