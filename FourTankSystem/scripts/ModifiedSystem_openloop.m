close all;
clear;
clc;

%% Parameters

p = FourTankStandardParams();

nx = 4;
nu = 2;
nh = 4;
ny = 2;
nz = 2;

%% Simulation scenario

t0 = 0;              % [s] Initial time
sim_min = 20;        % [min] Simulation duration
tf = sim_min*60;     % [s] Final time
Ts = 10;             % [s] Sampling time

t = (t0:Ts:tf)';
N = length(t);

%% Initial conditions

m10 = 0.0; % [g]
m20 = 0.0; % [g]
m30 = 0.0; % [g]
m40 = 0.0; % [g]

x0 = [m10;m20;m30;m40];

%% Piecewise-constant inputs sequences

% F1 and F2 are constant over each sampling interval.
%
% Example:
%   0 - 5 min    : F1 = 300, F2 = 300
%   5 - 10 min   : F1 = 350, F2 = 250
%   10 - 15 min  : F1 = 250, F2 = 350
%   15 - 20 min  : F1 = 300, F2 = 300

F1 = zeros(1,N);
F2 = zeros(1,N);

F1(t < 5*60) = 300;
F1(t >= 5*60 & t < 10*60) = 350;
F1(t >= 10*60 & t < 15*60) = 250;
F1(t >= 15*60) = 300;

F2(t < 5*60) = 300;
F2(t >= 5*60 & t < 10*60) = 250;
F2(t >= 10*60 & t < 15*60) = 350;
F2(t >= 15*60) = 300;

u = [F1;
     F2];

%% Fixed disturbances

disturbanceType = 'fixed';

[d, F3, F4] = GenerateDisturbances(t, Ts, disturbanceType, 1);
F3 = F3(1);
F4 = F4(1);

%% Simulation

x = zeros(nx,N);
h = zeros(nh,N);
y = zeros(ny,N);
z = zeros(nz,N);

x(:,1) = x0;

% Store the continuous-time trajectory for plotting if needed
T = [];
X = [];

for k = 1:N-1

    uk = u(:,k);
    dk = d(:,k);
    
    h(:,k) = FourTankSystemLevels(x(:,k),p);
    y(:,k) = FourTankSystemSensor(x(:,k),p);
    z(:,k) = FourTankSystemOutput(x(:,k),p);

    % Simulate the nonlinear system over one sampling interval
    [Tk,Xk] = ode15s(@(time,state) ModifiedFourTankSystem(time,state,uk,p,dk), [t(k) t(k+1)], x(:,k));

    % Store final state for the next sampling interval
    x(:,k+1) = Xk(end,:)';

    % Store continuous trajectory
    T = [T; Tk];
    X = [X; Xk];

end

% Compute outputs at final sampling instant
h(:,N) = FourTankSystemLevels(x(:,N),p);
y(:,N) = FourTankSystemSensor(x(:,N),p);
z(:,N) = FourTankSystemOutput(x(:,N),p);

%% Plot inputs and disturbances

figure;

subplot(2,1,1);
stairs(t/60,F1,'LineWidth',2);
hold on;
stairs(t/60,F2,'LineWidth',2);
xlabel('Time [min]');
ylabel('Pump flow [cm^3/s]');
legend('F_1','F_2');
title('Pump Inputs');
grid on;
xlim([0 sim_min]);

subplot(2,1,2);
stairs(t/60,F3*ones(size(t)),'LineWidth',2);
hold on;
stairs(t/60,F4*ones(size(t)),'LineWidth',2);
xlabel('Time [min]');
ylabel('Disturbance flow [cm^3/s]');
legend('F_3','F_4');
title('Fixed Disturbances');
grid on;
xlim([0 sim_min]);

%% Plot tank levels

figure;

for i = 1:4
    subplot(2,2,i);

    plot(t/60,h(i,:),'LineWidth',1.5, ...
         'DisplayName','True level');
    hold on;

    if i <= ny
        plot(t/60,y(i,:),'--','LineWidth',1.2, ...
     'DisplayName','Measured level');
    end

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    title(sprintf('Tank %d',i));
    legend('Location','best');
    grid on;
    xlim([0 sim_min]);
end

%% Plot all outputs together

figure;

plot(t/60,h','LineWidth',2);

xlabel('Time [min]');
ylabel('Tank level [cm]');
legend('h_1','h_2','h_3','h_4');
title('True Tank Levels');
grid on;
xlim([0 sim_min]);


%% Stochastic piecewise-constant disturbances

disturbanceType = 'piecewise';

[d, F3, F4] = GenerateDisturbances(t, Ts, disturbanceType, 1);

%% Simulation

x = zeros(nx,N);
h = zeros(nh,N);
y = zeros(ny,N);
z = zeros(nz,N);

x(:,1) = x0;

% Store continuous-time trajectory
T = [];
X = [];

for k = 1:N-1

    % Inputs and disturbances during [t_k,t_{k+1}]
    uk = u(:,k);
    dk = d(:,k);

    % Sensor measurements and controlled outputs
    h(:,k) = FourTankSystemLevels(x(:,k),p);
    y(:,k) = FourTankSystemSensorNoise(x(:,k),p);
    z(:,k) = FourTankSystemOutput(x(:,k),p);

    % Simulate nonlinear system over one sampling interval
    [Tk,Xk] = ode15s( ...
        @(time,state) ModifiedFourTankSystem(time,state,uk,p,dk), ...
        [t(k) t(k+1)], ...
        x(:,k));

    % Store final state
    x(:,k+1) = Xk(end,:)';

    % Store continuous trajectory
    T = [T; Tk];
    X = [X; Xk];

end

% Outputs at final sampling instant
h(:,N) = FourTankSystemLevels(x(:,N),p);
y(:,N) = FourTankSystemSensorNoise(x(:,N),p);
z(:,N) = FourTankSystemOutput(x(:,N),p);

%% Plot inputs

figure;

subplot(2,1,1);
stairs(t/60,F1,'LineWidth',2);
hold on;
stairs(t/60,F2,'LineWidth',2);
xlabel('Time [min]');
ylabel('Pump flow [cm^3/s]');
legend('F_1','F_2');
title('Pump Inputs');
grid on;
xlim([0 sim_min]);

subplot(2,1,2);
stairs(t/60,F3,'LineWidth',2);
hold on;
stairs(t/60,F4,'LineWidth',2);
xlabel('Time [min]');
ylabel('Disturbance flow [cm^3/s]');
legend('F_3','F_4');
title('Piecewise-Constant Stochastic Disturbances');
grid on;
xlim([0 sim_min]);

%% Plot tank levels

figure;

for i = 1:4
    subplot(2,2,i);

    plot(t/60,h(i,:),'LineWidth',1.5, ...
         'DisplayName','True level');
    hold on;

    if i <= ny
        plot(t/60,y(i,:),'--','LineWidth',1.2, ...
     'DisplayName','Measured level');
    end

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    title(sprintf('Tank %d',i));
    legend('Location','best');
    grid on;
    xlim([0 sim_min]);
end

%% Plot all outputs together

figure;

plot(t/60,h','LineWidth',2);

xlabel('Time [min]');
ylabel('Tank level [cm]');
legend('h_1','h_2','h_3','h_4');
title('True Tank Levels');
grid on;
xlim([0 sim_min]);



%% Langevin disturbance parameters

disturbanceType = 'langevin';

[d, F3, F4] = GenerateDisturbances(t, Ts, disturbanceType, 1);

%% Random number generator

rng(1);

%% Simulation

x = zeros(nx,N);
h = zeros(nh,N);
y = zeros(ny,N);
z = zeros(nz,N);

x(:,1) = x0;

for k = 1:N-1

    % Outputs at current sampling instant
    h(:,k) = FourTankSystemLevels(x(:,k),p);

    % True controlled outputs
    z(:,k) = FourTankSystemOutput(x(:,k),p);

    % Noisy measurements
    y(:,k) = FourTankSystemSensorNoise(x(:,k),p);

    % System simulation

    uk = u(:,k);
    dk = d(:,k);

    [Tk,Xk] = ode15s( ...
        @(time,state) ModifiedFourTankSystem( ...
            time,state,uk,p,dk), ...
        [t(k) t(k+1)], ...
        x(:,k));

    x(:,k+1) = Xk(end,:)';

end

% Outputs at final sampling instant
h(:,N) = FourTankSystemLevels(x(:,N),p);
z(:,N) = FourTankSystemOutput(x(:,N),p);
y(:,N) = FourTankSystemSensorNoise(x(:,N),p);

% Extract disturbances

F3 = d(1,:);
F4 = d(2,:);

%% Plot inputs and disturbances

figure;

subplot(2,1,1);

stairs(t/60,F1,'LineWidth',2);
hold on;
stairs(t/60,F2,'LineWidth',2);

xlabel('Time [min]');
ylabel('Pump flow [cm^3/s]');
legend('F_1','F_2');
title('Pump Inputs');
grid on;
xlim([0 sim_min]);

subplot(2,1,2);

plot(t/60,F3,'LineWidth',1.5);
hold on;
plot(t/60,F4,'LineWidth',1.5);

xlabel('Time [min]');
ylabel('Disturbance flow [cm^3/s]');
legend('F_3','F_4');
title('Langevin Disturbances');
grid on;
xlim([0 sim_min]);

%% Plot true and measured tank levels

figure;

for i = 1:4
    subplot(2,2,i);

    plot(t/60,h(i,:),'LineWidth',1.5, ...
         'DisplayName','True level');
    hold on;

    if i <= ny
        plot(t/60,y(i,:),'--','LineWidth',1.2, ...
     'DisplayName','Measured level');
    end

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    title(sprintf('Tank %d',i));
    legend('Location','best');
    grid on;
    xlim([0 sim_min]);
end

%% Plot all true outputs

figure;

plot(t/60,h','LineWidth',2);

xlabel('Time [min]');
ylabel('Tank level [cm]');
legend('h_1','h_2','h_3','h_4');
title('True Tank Levels');
grid on;
xlim([0 sim_min]);