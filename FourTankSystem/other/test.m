function test
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
end