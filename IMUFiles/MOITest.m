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
I_11W = ;
omega = ;
omega_dot = ;

n = size(p);
for i = 1:n
    A(:,:,i) = [p_dot(i) q_dot(i)-(p(i)*r(i)) r_dot(i)+p(i)*q(i) -q(i)*r(i) (q(i)^2)-(r(i)^2) q(i)*r(i);
        p(i)*r(i) p_dot(i)+q(i)*r(i) (r(i)^2)-(p(i)^2) q_dot r_dot-p(i)*q(i) -p(i)*r(i);
        -p(i)*q(i) (p(i)^2)-(q(i)^2) p_dot(i)-q(i)*r(i) p(i)*q(i) q_dot(i)+p(i)*r(i) r_dot(i)]; % substituted p_dot for d_dot in (2,2) because not sure what d is
    
    T = []
    
    A_pseudo(:,:,i) = ((A'(:,:,i)A(:,:,i))^(-1))*A'(:,:,i)
    x(:,:,i) = A_pseudo(:,:,i)*T(:,:,i);
end


I_11 = x(1,1,i);
I_12 = x(1,2,i);
I_13 = x(1,3,i);
I_22 = x(1,4,i);
I_23 = x(1,5,i);
I_33 = x(1,6,i);

% plots
plot(n,I_11)% plot I_11 vs nth step
plot(n,I_12)
plot(n,I_13)
plot(n,I_22)
plot(n,I_23)
plot(n,I_33)
