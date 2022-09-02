%% Magnetorquer inductive spike reduction circuit calcs
%  Mike Stromecki 6/3/2022
%  Magnetorquer values 

%% Electrical Measurements
%  Ohms as connected
%  Using Fluke 45 bench DMM 
%  Probe pins as soldered on MTQ board with nothing else connected.
%  (Pre- resistor addition) April 5, 2022
%  MTQ 1	26.50 Ohms
%  MTQ 2	27.40 (26.7 on mtq connector pins)
%  MTQ 3	26.58


%% Required Resistors and Power Dissipated Calcs
V = 5;
R = [26.5 27.4 26.58];
I = V./R;   % = [0.1887    0.1825    0.1881] A

% Max current - 140 mA
I2 = 0.140;
R = V./I;
R2 = V/I2;  % = 35.7143 
Rd = R2-R;  % = [9.2143    8.3143    9.1343] Ohms more on each

P = I.*V;   % = (I^2).*R
Pdis = (I.^2).*Rd;  % = [0.1806    0.1630    0.1790] 
        % W dissipated in each resistor at full 5V .140 A
 
%% RLC Protection Calcs
% RC in series, in parallel with L
L = 0.2161;     % H according to calcs in MTQ_Design Line 308
I_max = 0.140;  % mA
V_max = 10;     % VDC This is the voltage allowable across the switch 
                % immediately after opening.

E_ind = 0.5*L*I_max^2;  % energy stored in inductor coil.
% E_cap = E_ind = 0.5*C*V^2; % energy in capacitor. 
C = E_ind*2/(V_max^2);

R_c = V_max/I_max;      % in series with cap = R=V/I

C_c = [39 47 56]*1e-6; % Potential other capacitor values.
V_c = 10;           % V_rated of other potential caps.
E_cap = 0.5.*C_c.*V_c.^2; % energy in capacitor. 
% E_cap = 1.9500e-003   2.3500e-003    2.8000e-003

fprintf("Imax, Vmax, L, E, C, Rc \n");
fprintf("%g %g %g %g %g %g \n", I_max, V_max, L, E_ind, C, R_c);

% Imax  Vmax    L      E         C        Rc
% 0.14   10  0.2161 2.118e-3 42.3556e-6 71.4286