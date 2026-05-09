%EE516 Power Systems
% Mulualem Ayena
%elctromagnet
r = 10;                 % Coil resistance [Ohms]
k = 6.283e-5;           % Inductance coefficient [H·m]
x0 = 0.003;             % Initial position/spring reference [m] (3 mm)
K = 2667;               % Spring constant [N/m]
D = 4;                  % Damping coefficient [N·s/m]
M = 0.055;              % Mass [kg]

% Voltage profile: 5V from 0 to 0.3s, then 0V 
v_func = @(t) 5 * (t >= 0 & t < 0.3);

% Force profile: 4N from 0.15 to 0.3s, otherwise 0 
fm_func = @(t) 4 * (t >= 0.15 & t < 0.3);

%%  SYSTEM ODEs 
% State vector: y(1) = i (current), y(2) = x (position), y(3) = v_x (velocity)
system_ode = @(t, y) [
    % di/dt = (x/k)*(v - r*i) + (i*v_x)/x
    (y(2)/k) * (v_func(t) - r*y(1)) + (y(1)*y(3))/y(2);
    
    % dx/dt = v_x
    y(3);
    
    % dv_x/dt = (1/M)*(f_m + f_e - D*v_x - K*(x - x0))
    % where f_e = -k*i^2/(2*x^2)
    (1/M) * (fm_func(t) - k*y(1)^2/(2*y(2)^2) - D*y(3) - K*(y(2) - x0))
];
%%  SIMULATION SETTINGS 
t_start = 0;
t_end = 0.5;           
t_span = [t_start t_end];

% Initial conditions
i0 = 0;                 % Initial current [A]
x_init = 0.003;         % Initial position [m] (3 mm)
v_x0 = 0;               % Initial velocity [m/s]
y0 = [i0; x_init; v_x0];

% Solve using ode45 with high resolution
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.0005);
[t, y] = ode45(system_ode, t_span, y0, options);

i = y(:, 1);            % Current [A]
x = y(:, 2);            % Position [m]
v_x = y(:, 3);          % Velocity [m/s]

% Calculate derived quantities
f_e = -k * i.^2 ./ (2 * x.^2);           % Electromagnetic force [N]
L = k ./ x;                               % Inductance [H]

% Calculate power quantities
p_source = v_func(t) .* i;                 % Instantaneous input power
p_resistor = r * i.^2;                     % Power dissipated in resistor
p_field = gradient(0.5 * L .* i.^2, t);    % Power into magnetic field
p_mech = -f_e .* v_x;                      % Mechanical power

% Energy calculations
W_field = 0.5 * L .* i.^2 * 1000;          % Magnetic field energy [mJ]
W_spring = 0.5 * K * (x - x0).^2 * 1000;    % Spring energy [mJ]
W_kinetic = 0.5 * M * v_x.^2 * 1000;        % Kinetic energy [mJ]
%%  PLOT 1: Combined Main Results 
figure('Position', [100 100 1400 1000]);

% Current
subplot(2,2,1);
plot(t, i, 'g-', 'LineWidth', 1);
hold on;
xlabel('time (seconds)', 'FontSize', 12);
ylabel('Current (A)', 'FontSize', 12);
title('(a)  Current', 'FontSize', 14);
grid on;
xlim([0 0.5]);
ylim([0 0.6]);

% Position
subplot(2,2,2);
plot(t, x*1000, 'b-', 'LineWidth', 1);
hold on;
line([0 0.5], [3 3], 'Color', 'k', 'LineStyle', ':');
xlabel('time (seconds)', 'FontSize', 12);
ylabel('Position (mm)', 'FontSize', 12);
title('(b)  Position', 'FontSize', 14);
grid on;
xlim([0 0.5]);
ylim([2.0 6.0]);

% Velocity
subplot(2,2,3);
plot(t, v_x*1000, 'g-', 'LineWidth', 1);
hold on;
line([0 0.5], [0 0], 'Color', 'k', 'LineStyle', ':');
xlabel('time (seconds)', 'FontSize', 12);
ylabel('Velocity (mm/s)', 'FontSize', 12);
title('(c) Velocity', 'FontSize', 14);
grid on;
xlim([0 0.5]);
ylim([-300 300]);

% Electromagnetic Force
subplot(2,2,4);
plot(t, f_e, 'c-', 'LineWidth', 1);
hold on;
line([0 0.5], [0 0], 'Color', 'k', 'LineStyle', ':');
xlabel('Time (Seconds)', 'FontSize', 12);
ylabel('Force (N)', 'FontSize', 12);
title('(d) Electromagnetic Force', 'FontSize', 14);
grid on;
xlim([0 0.5]);
ylim([-1.5 0]);

sgtitle('Electromagnet Response to Applied Voltage and Force', 'FontSize', 16, 'FontWeight', 'bold');

%%  PLOT 2: Inductance and Power 
figure('Position', [100 100 1200 800]);
% Inductance
subplot(2,2,1);
plot(t, L*1000, 'g-', 'LineWidth', 1);
hold on;
xlabel('Time (Seconds)', 'FontSize', 12);
ylabel('Inductance (mH)', 'FontSize', 12);
title('Coil Inductance L = k/x', 'FontSize', 14);
grid on;
xlim([0 0.5]);

% Input Power
subplot(2,2,2);
plot(t, p_source, 'b-', 'LineWidth', 1);
hold on;
xlabel('Time (Seconds)', 'FontSize', 12);
ylabel('Inst. P in r (W)', 'FontSize', 10);
grid on;
xlim([0 0.5]);
ylim([0 3]);

% Resistor Power
subplot(2,2,3);
plot(t, p_resistor, 'b-', 'LineWidth', 1);
hold on;
xlabel('Time (Seconds)', 'FontSize', 12);
ylabel('Inst. input Power (W)', 'FontSize', 10);
grid on;
xlim([0 0.5]);
ylim([0 3.5]);
