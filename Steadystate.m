%EE516 Power Systems
% Mulualem Ayena
%Steady state
r  = 10;                 % Ohms
k  = 6.283e-5;           % H*m
x0 = 3e-3;               % Initial/held position [m] (3 mm)
K = 2667;               % Spring constant [N/m]
D = 4;                  % Damping coefficient [N·s/m]
M = 0.055;              % Mass [kg]% meters (fixed position)
v = @(t) 5;              % constant 5 V for t > 0

odefun = @(t, i) (x0/k) * (v(t) - r*i);

i0 = 0;

tspan = [0 0.5];

[t, i] = ode45(odefun, tspan, i0);

fe = -(k .* i.^2) ./ (2 * x0^2);

figure;
subplot(2,1,1)
plot(t, i, 'g', 'LineWidth', 1)
grid on
xlabel('Time (s)')
ylabel('Current (A)')
title('Current')

subplot(2,1,2)
plot(t, fe, 'b', 'LineWidth', 1)
grid on
xlabel('Time (s)')
ylabel('Force (N)')
title('Electromagnetic Force')