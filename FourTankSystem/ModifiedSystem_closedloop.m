close all;
clear;
clc;

%% Parameters

p = FourTankStandardParams();

F3 = 50;
F4 = 30;

d = [F3; F4];

%% Simulation Settings

Ts = 10;
sim_min = 20;
tf = sim_min*60;

t = (0:Ts:tf)';
N = length(t);

nx = 4; nu = 2; ny = 4; nz = 2;

%% Initial conditions and setpoints

x0 = zeros(nx,1); % Initially empty tanks

zbar = [30;25]; % Setpoints for Tank 1 and Tank 2 heights [cm]

umin = 0; 
umax = 1000; 

us = [250;250]; % Steady-state inputs (nominal flow rates) [cm^3/s]

%% Controller parameters

Kp = [10;10];

Ti_PI  = [40;40];
Ti_PID = [40;40];

Td_PID = [5;5];

controllerTypes = {'P','PI','PID'};

% Structure for storing results
results = struct();

%% Run simulations for P, PI, and PID

for j = 1:length(controllerTypes)

    controllerType = controllerTypes{j};

    switch controllerType
        case 'P'
            Ti = [Inf;Inf];
            Td = [0;0];

        case 'PI'
            Ti = Ti_PI;
            Td = [0;0];

        case 'PID'
            Ti = Ti_PID;
            Td = Td_PID;
    end

    % Initialize controller states
    i_state = zeros(2,1);
    z_prev = zeros(2,1);

    % Initialize simulation variables
    x = zeros(nx,N);
    y = zeros(ny,N);
    z = zeros(nz,N);
    u = zeros(nu,N);

    X = zeros(0,nx);
    T = zeros(0,1);

    x(:,1) = x0;

    % Simulation loop
    for k = 1:N-1

        % Sensor function and output function
        y(:,k) = FourTankSystemSensor(x(:,k),p);
        z(:,k) = FourTankSystemOutput(x(:,k),p);

        Ts_k = t(k+1) - t(k);
        [u(:,k), i_state, z_prev] = PIDControl(i_state, z_prev, zbar, z(1:2,k), us, Kp, Ti, Td, Ts_k, umin, umax, controllerType);

        % Simulate from time t[k] to time t[k+1]
        odefun = @(t,x) ModifiedFourTankSystem(t,x,u(:,k),p,d);
        [Tk,Xk] = ode45(odefun,[t(k) t(k+1)],x(:,k));

        x(:,k+1) = Xk(end,:)';

        % Store simulated results (for plotting)
        T = [T; Tk];
        X = [X; Xk];
    end

    % Evaluate final sample
    k = N;

    y(:,k) = FourTankSystemSensor(x(:,k),p);
    z(:,k) = FourTankSystemOutput(x(:,k),p);
    u(:,k) = u(:,k-1);

    % Store results for this controller
    results.(controllerType).t = t;
    results.(controllerType).x = x;
    results.(controllerType).y = y;
    results.(controllerType).z = z;
    results.(controllerType).u = u;

end

%% Plotting Results: P, PI, and PID Controllers

figure("Color","w");

% --- Top Left: Tank 1 Level ---
subplot(2,2,1);

plot(t/60, results.P.z(1,:), 'r--', 'LineWidth', 1.5);
hold on;
plot(t/60, results.PI.z(1,:), 'b-.', 'LineWidth', 1.5);
plot(t/60, results.PID.z(1,:), 'LineWidth', 1.5);
yline(zbar(1), 'k:', 'LineWidth', 1.5);

title('Tank 1');
xlabel('Time [min]');
ylabel('h_1 [cm]');
legend('P','PI','PID','Setpoint','Location','best');
grid on;
xlim([0 t(end)/60]);

% --- Top Right: Tank 2 Level ---
subplot(2,2,2);

plot(t/60, results.P.z(2,:), 'r--', 'LineWidth', 1.5);
hold on;
plot(t/60, results.PI.z(2,:), 'b-.', 'LineWidth', 1.5);
plot(t/60, results.PID.z(2,:), 'LineWidth', 1.5);
yline(zbar(2), 'k:', 'LineWidth', 1.5);

title('Tank 2');
xlabel('Time [min]');
ylabel('h_2 [cm]');
legend('P','PI','PID','Setpoint','Location','best');
grid on;
xlim([0 t(end)/60]);

% --- Bottom Left: Input Flow F1 ---
subplot(2,2,3);

plot(t/60, results.P.u(1,:), 'r--', 'LineWidth', 1.5);
hold on;
plot(t/60, results.PI.u(1,:), 'b-.', 'LineWidth', 1.5);
plot(t/60, results.PID.u(1,:), 'LineWidth', 1.5);

title('Pump 1 Input');
xlabel('Time [min]');
ylabel('F_1 [cm^3/s]');
legend('P','PI','PID','Location','best');
grid on;
xlim([0 t(end)/60]);

% --- Bottom Right: Input Flow F2 ---
subplot(2,2,4);

plot(t/60, results.P.u(2,:), 'r--', 'LineWidth', 1.5);
hold on;
plot(t/60, results.PI.u(2,:), 'b-.', 'LineWidth', 1.5);
plot(t/60, results.PID.u(2,:), 'LineWidth', 1.5);

title('Pump 2 Input');
xlabel('Time [min]');
ylabel('F_2 [cm^3/s]');
legend('P','PI','PID','Location','best');
grid on;
xlim([0 t(end)/60]);

sgtitle('Comparison of P, PI, and PID Controllers');