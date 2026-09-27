close all; clear; clc;
%% Four Tank Simulation - Open Loop
% Lecture 1
%% Parameters

a1 = 1.2272; %[cm2] Area of outlet pipe 1
a2 = 1.2272; %[cm2] Area of outlet pipe 2
a3 = 1.2272; %[cm2] Area of outlet pipe 3
a4 = 1.2272; %[cm2] Area of outlet pipe 4
A1 = 380.1327; %[cm2] Cross sectional area of tank 1
A2 = 380.1327; %[cm2] Cross sectional area of tank 2
A3 = 380.1327; %[cm2] Cross sectional area of tank 3
A4 = 380.1327; %[cm2] Cross sectional area of tank 4


gamma1 = 0.45; % Flow distribution constant. Valve 1
gamma2 = 0.40; % Flow distribution constant. Valve 2

g = 981; %[cm/s2] The acceleration of gravity
rho = 1.00; %[g/cm3] Density of water

p = [a1;a2;a3;a4; A1;A2;A3;A4; gamma1;gamma2; g; rho];

nx = 4; nu = 2; ny = 4; nz = 2;


%% Simulation scenario

t0 = 0.0; % [s] Initial time
sim_min = 20;
tf = sim_min*60; % [s] Final time

Ts = 10; % [s] Sample time

t = [t0:Ts:tf]'; % [s] Sample instants
N = length(t);

m10 = 0.0; % [g] Liquid mass in tank 1 at time t0
m20 = 0.0; % [g] Liquid mass in tank 2 at time t0
m30 = 0.0; % [g] Liquid mass in tank 3 at time t0
m40 = 0.0; % [g] Liquid mass in tank 4 at time t0

F1 = 300; % [cm3/s] Flow rate from pump 1
F2 = 300; % [cm3/s] Flow rate from pump 2




x0 = [m10; m20; m30; m40];
u = [F1; F2];


%% Compute the solution / Simulate Open Loop

% Solve the system of differential equations
[T,X] = ode15s(@FourTankSystem,[t0 tf],x0,[],u,p);

% help variables
[nT,nX] = size(X);
a = p(1:4,1)';
A = p(5:8,1)';

% Compute the measured variables
H = zeros(nT,nX);
for i=1:nT
H(i,:) = X(i,:)./(rho*A);
end

% Compute the flows out of each tank
Qout = zeros(nT,nX);
for i=1:nT
Qout(i,:) = a.*sqrt(2*g*H(i,:));
end
%% Plot
close all;
% Create a new figure
figure;

% --- Subplot 1: h3 ---
subplot(2, 2, 1);
plot(T/60, H(:, 3), 'b', 'LineWidth', 2);
xlabel('time [min]');
ylabel('h3 [cm]');
xlim([0, sim_min]);
ylim([0, 12]);
grid on;

% --- Subplot 2: h4 ---
subplot(2, 2, 2);
plot(T/60, H(:, 4), 'b', 'LineWidth', 2);
xlabel('time [min]');
ylabel('h4 [cm]');
xlim([0, sim_min]);
ylim([0, 12]);
grid on;

% --- Subplot 3: h1 ---
subplot(2, 2, 3);
plot(T/60, H(:, 1), 'b', 'LineWidth', 2);
xlabel('time [min]');
ylabel('h1 [cm]');
xlim([0, sim_min]);
ylim([0, 40]);
grid on;

% --- Subplot 4: h2 ---
subplot(2, 2, 4);
plot(T/60, H(:, 2), 'b', 'LineWidth', 2);
xlabel('time [min]');
ylabel('h2 [cm]');
xlim([0, sim_min]);
ylim([0, 40]);
grid on;


%% Simulate in discrete steps 


x = zeros(nx,N);
y = zeros(ny,N);
z = zeros(nz,N);

X = zeros(0,nx);
T = zeros(0,1);

x(:,1) = x0; % Initial condition
u = [repmat(F1,1,N); repmat(F2,1,N)];

for k = 1:N-1
    % Sensor function and output function
    y(:,k) = FourTankSystemSensor(x(:,k),p);
    z(:,k) = FourTankSystemOutput(x(:,k),p);
    
    % Simulate from time t[k] to time t[k+1]
    [Tk,Xk]=ode15s(@FourTankSystem,[t(k) t(k+1)],x(:,k),[],u(:,k),p);
    x(:,k+1) = Xk(end,:)';
    
    % Store the simulated results (for plotting)
    T = [T; Tk];
    X = [X; Xk];
end

k = N;
% Sensor function and output function
y(:,k) = FourTankSystemSensor(x(:,k),p);
z(:,k) = FourTankSystemOutput(x(:,k),p);

%% Plotting Results
figure;

% --- Top Left: Tank 1 Level ---
subplot(2,2,1);
plot(t, z(1,:), 'b-', 'LineWidth', 2);
title('Tank 1');
xlabel('time [min]');
ylabel('h_1 [cm]');
grid on;
xlim([0 t(end)]);

% --- Top Right: Tank 2 Level ---
subplot(2,2,2);
plot(t, z(2,:), 'b-', 'LineWidth', 2);
title('Tank 2');
xlabel('time [min]');
ylabel('h_2 [cm]');
grid on;
xlim([0 t(end)]);

% --- Bottom Left: Input Flow F1 ---
subplot(2,2,3);
plot(t, u(1,:), 'b-', 'LineWidth', 2);
xlabel('time [min]');
ylabel('F_1 [cm^3/s]');
grid on;
xlim([0 t(end)]);

% --- Bottom Right: Input Flow F2 ---
subplot(2,2,4);
plot(t, u(2,:), 'b-', 'LineWidth', 2);
xlabel('time [min]');
ylabel('F_2 [cm^3/s]');
grid on;
xlim([0 t(end)]);
