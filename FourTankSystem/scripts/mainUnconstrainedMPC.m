clear;
clc;
close all;

%% Load the linearized discrete-time model

data = load('linearModel.mat');

A = data.Adisc;
B = data.Bdisc;
Cz = data.Cdisc;
Ts = data.Ts;

nx = size(A,1);
nu = size(B,2);
nz = size(Cz,1);

%% Design the MPC

Npred = 60;  % Prediction horizon: 60*5 = 300 s

% Slides use weights inside squared norms
Wz = eye(nz);
Wu = sqrt(1e-3)*eye(nu);
Wdu = sqrt(1e-3)*eye(nu);

mpc = DesignUnconstrainedMPC(A,B,Cz,Npred,Wz,Wu,Wdu);

%% Simulation settings : LINEAR model for MPC and simulation

t = (0:Ts:20*60)';
N = numel(t);

% Absolute level setpoints [cm]
zSetpoint = data.yss + [3;2];

% Reference in deviation variables
r = zSetpoint - data.yss;

% States and commands are stored as deviations
X = zeros(nx,N);
U = zeros(nu,N-1);

ubar = zeros(nu,1);
uPrevious = zeros(nu,1);

% Start at nominal equilibrium
X(:,1) = zeros(nx,1);

% Save the first predicted trajectory for illustration
firstPrediction = [];

%% Closed-loop simulation

for k = 1:N-1

    % Optimize the future commands
    [uk,Uoptimal,Zpredicted] = ComputeUnconstrainedMPC(mpc,X(:,k),r,ubar,uPrevious);

    % Keep the first plan for plotting
    if k == 1
        firstPrediction = reshape(Zpredicted,nz,Npred);
    end

    % Apply only the first command of the plan
    U(:,k) = uk;

    % Simulate one sampling interval
    X(:,k+1) = A*X(:,k) + B*uk;

    uPrevious = uk;
end

%% Absolute levels and pump flows

Zabsolute = data.yss + Cz*X;
Uabsolute = data.us + U;

%% Plot the closed-loop response

figure('Color','w');

for i = 1:nz
    subplot(2,2,i);

    plot(t/60,Zabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(zSetpoint(i),'k--');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Actual level','Setpoint','Location','best');
    grid on;
end

for i = 1:nu
    subplot(2,2,2+i);

    stairs(t(1:end-1)/60,Uabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(data.us(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('F_%d [cm^3/s]',i));
    legend('Applied flow','Nominal flow','Location','best');
    grid on;
end

sgtitle('Unconstrained MPC: linear plant and known states');

%% Plot the prediction made at the first instant

tPrediction = (1:Npred)*Ts;

figure('Color','w');

for i = 1:nz
    subplot(nz,1,i);

    plot(tPrediction/60, ...
         data.yss(i) + firstPrediction(i,:), ...
         '--','LineWidth',1.5);
    hold on;

    plot(t/60,Zabsolute(i,:),'LineWidth',1.3);
    yline(zSetpoint(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Initial prediction','Closed-loop response', ...
           'Setpoint','Location','best');

    xlim([0 Npred*Ts/60]);
    grid on;
end


%% Simulation settings : LINEAR model for MPC, NON-LINEAR for simulation

t = (0:Ts:20*60)';
N = numel(t);

% Absolute level setpoints [cm]
zSetpoint = data.yss + [3;2];

% Reference in deviation variables
r = zSetpoint - data.yss;

% States and commands are stored as deviations
X = zeros(nx,N);
U = zeros(nu,N-1);

ubar = zeros(nu,1);
uPrevious = zeros(nu,1);

% Start at nominal equilibrium
xActual = data.xss;

X(:,1) = xActual - data.xss;

% Save the first predicted trajectory for illustration
firstPrediction = [];

%% Closed-loop simulation

for k = 1:N-1

    % LINEAR MODEL: predict and optimize
    xDeviation = xActual - data.xss;

    [uk,Uoptimal,Zpredicted] = ComputeUnconstrainedMPC(mpc,xDeviation,r,ubar,uPrevious);

    % Store the first predicted output trajectory
    if k == 1
        firstPrediction = reshape(Zpredicted,nz,Npred);
    end

    % Store command deviation
    U(:,k) = uk;

    % NONLINEAR MODEL: simulate the applied command
    uActual = data.us + uk;
    
    [~,Xinterval] = ode15s( ...
        @(time,state) ModifiedFourTankSystem( ...
            time,state,uActual,data.p,data.d),[t(k) t(k+1)],xActual);
    
    xActual = Xinterval(end,:)';

    % Store the resulting state deviation
    X(:,k+1) = xActual - data.xss;

    % Previous command deviation for the movement penalty
    uPrevious = uk;
end

%% Absolute levels and pump flows

Zabsolute = data.yss + Cz*X;
Uabsolute = data.us + U;

%% Plot the closed-loop response

figure('Color','w');

for i = 1:nz
    subplot(2,2,i);

    plot(t/60,Zabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(zSetpoint(i),'k--');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Actual level','Setpoint','Location','best');
    grid on;
end

for i = 1:nu
    subplot(2,2,2+i);

    stairs(t(1:end-1)/60,Uabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(data.us(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('F_%d [cm^3/s]',i));
    legend('Applied flow','Nominal flow','Location','best');
    grid on;
end

sgtitle('Unconstrained MPC: nonlinear plant and known states');

%% Plot the prediction made at the first instant

tPrediction = (1:Npred)*Ts;

figure('Color','w');

for i = 1:nz
    subplot(nz,1,i);

    plot(tPrediction/60, ...
         data.yss(i) + firstPrediction(i,:), ...
         '--','LineWidth',1.5);
    hold on;

    plot(t/60,Zabsolute(i,:),'LineWidth',1.3);
    yline(zSetpoint(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Initial prediction','Closed-loop response', ...
           'Setpoint','Location','best');

    xlim([0 Npred*Ts/60]);
    grid on;
end


%% LINEAR model for MPC and simulation + KALMAN FILTER

% Stationary Kalman filter

C = data.Cdisc;
G = data.Edisc;

sigmaProcess = 5;        % Assumed flow uncertainty [cm^3/s]
sigmaMeasurement = 0.1;  % Sensor noise standard deviation [cm]

Q = sigmaProcess^2 * eye(2);
R = sigmaMeasurement^2 * eye(2);

[Kfx,~,~] = DesignStationaryKalmanFilter(A,G,C,Q,R);

rng(1);

%% Simulation settings

t = (0:Ts:20*60)';
N = numel(t);

% True and estimated state deviations
X = zeros(nx,N);
Xestimated = zeros(nx,N);

% Measured level deviations
Y = zeros(nz,N);

% Applied command deviations
U = zeros(nu,N-1);

uPrevious = zeros(nu,1);

% True absolute plant state
xActual = data.xss;

% Initial predicted state deviation
xPredicted = zeros(nx,1);

firstPrediction = [];

for k = 1:N

    % 1. Simulated sensors: only h1 and h2

    yMeasured = FourTankSystemSensorNoise( ...
        xActual,data.p,sigmaMeasurement);

    yDeviation = yMeasured - data.yss;

    % 2. Kalman measurement update

    innovation = yDeviation - C*xPredicted;
    xFiltered = xPredicted + Kfx*innovation;

    % Store values for evaluation and plotting
    X(:,k) = xActual - data.xss;
    Xestimated(:,k) = xFiltered;
    Y(:,k) = yDeviation;

    if k < N

        % 3. MPC uses the ESTIMATED state

        [uk,Uoptimal,Zpredicted] = ComputeUnconstrainedMPC( ...
            mpc,xFiltered,r,ubar,uPrevious);

        U(:,k) = uk;

        if k == 1
            firstPrediction = reshape(Zpredicted,nz,Npred);
        end

        % 4. Apply the command to the nonlinear plant

        uActual = data.us + uk;

        [~,Xinterval] = ode15s( ...
            @(time,state) ModifiedFourTankSystem( ...
                time,state,uActual,data.p,data.d), ...
            [t(k) t(k+1)],xActual);

        xActual = Xinterval(end,:)';

        % 5. Kalman prediction for the next measurement

        xPredicted = A*xFiltered + B*uk;

        uPrevious = uk;
    end
end

%% Absolute levels and flows

Zabsolute = data.yss + Cz*X;
Zestimated = data.yss + Cz*Xestimated;
Yabsolute = data.yss + Y;

Uabsolute = data.us + U;

%% Plot the closed-loop response

figure('Color','w');

for i = 1:nz
    subplot(2,2,i);

    plot(t/60,Zabsolute(i,:),'LineWidth',1.5);
    hold on;
    
    plot(t/60,Zestimated(i,:),'--','LineWidth',1.2);
    
    yline(zSetpoint(i),'k:');
    
    legend('True level','Estimated level','Setpoint', ...
           'Location','best');
    grid on;
end

for i = 1:nu
    subplot(2,2,2+i);

    stairs(t(1:end-1)/60,Uabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(data.us(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('F_%d [cm^3/s]',i));
    legend('Applied flow','Nominal flow','Location','best');
    grid on;
end

sgtitle('Unconstrained MPC: linear plant and known states');

%% Plot the prediction made at the first instant

tPrediction = (1:Npred)*Ts;

figure('Color','w');

for i = 1:nz
    subplot(nz,1,i);

    plot(tPrediction/60, ...
         data.yss(i) + firstPrediction(i,:), ...
         '--','LineWidth',1.5);
    hold on;

    plot(t/60,Zabsolute(i,:),'LineWidth',1.3);
    yline(zSetpoint(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Initial prediction','Closed-loop response', ...
           'Setpoint','Location','best');

    xlim([0 Npred*Ts/60]);
    grid on;
end

%% Evaluate state estimation

rho = data.p(12);
Atank = data.p(5:8);

HallTrue = (data.xss + X) ./ (rho*Atank);
HallEstimated = (data.xss + Xestimated) ./ (rho*Atank);

figure('Color','w');

for i = 1:4
    subplot(2,2,i);

    plot(t/60,HallTrue(i,:),'k','LineWidth',1.4);
    hold on;
    plot(t/60,HallEstimated(i,:),'--','LineWidth',1.2);

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('True','Estimated','Location','best');
    grid on;
end

sgtitle('Kalman estimation during MPC control');



%% LINEAR model for MPC and simulation + KALMAN FILTER

% Dynamic Kalman filter

C = data.Cdisc;
G = data.Edisc;

sigmaProcess = 5;        % Assumed flow uncertainty [cm^3/s]
sigmaMeasurement = 0.1;  % Sensor noise standard deviation [cm]

Q = sigmaProcess^2 * eye(2);
R = sigmaMeasurement^2 * eye(2);

% Initial uncertainty: 1 cm standard deviation per tank
rho = data.p(12);
Atank = data.p(5:8);

sigmaMass0 = rho * Atank * 1;
Ppredicted = diag(sigmaMass0.^2);

rng(1);

%% Simulation settings

t = (0:Ts:20*60)';
N = numel(t);

% True and estimated state deviations
X = zeros(nx,N);
Xestimated = zeros(nx,N);

% Measured level deviations
Y = zeros(nz,N);

% Applied command deviations
U = zeros(nu,N-1);

uPrevious = zeros(nu,1);

% True absolute plant state
xActual = data.xss;

% Initial predicted state deviation
xPredicted = zeros(nx,1);

firstPrediction = [];

for k = 1:N

    % 1. Simulated sensors: only h1 and h2

    yMeasured = FourTankSystemSensorNoise( ...
        xActual,data.p,sigmaMeasurement);

    yDeviation = yMeasured - data.yss;

    % 2. Kalman measurement update

    innovation = yDeviation - C*xPredicted;
    Re = C*Ppredicted*C' + R;
    Kfx = (Ppredicted*C') / Re;
    
    xFiltered = xPredicted + Kfx*innovation;
    Pfiltered = Ppredicted - Kfx*Re*Kfx';

    % Store values for evaluation and plotting
    X(:,k) = xActual - data.xss;
    Xestimated(:,k) = xFiltered;
    Y(:,k) = yDeviation;

    if k < N

        % 3. MPC uses the ESTIMATED state

        [uk,Uoptimal,Zpredicted] = ComputeUnconstrainedMPC( ...
            mpc,xFiltered,r,ubar,uPrevious);

        U(:,k) = uk;

        if k == 1
            firstPrediction = reshape(Zpredicted,nz,Npred);
        end

        % 4. Apply the command to the nonlinear plant

        uActual = data.us + uk;

        [~,Xinterval] = ode15s( ...
            @(time,state) ModifiedFourTankSystem( ...
                time,state,uActual,data.p,data.d), ...
            [t(k) t(k+1)],xActual);

        xActual = Xinterval(end,:)';

        % 5. Kalman prediction for the next measurement

        xPredicted = A*xFiltered + B*uk;
        Ppredicted = A*Pfiltered*A' + G*Q*G';

        uPrevious = uk;
    end
end

%% Absolute levels and flows

Zabsolute = data.yss + Cz*X;
Zestimated = data.yss + Cz*Xestimated;
Yabsolute = data.yss + Y;

Uabsolute = data.us + U;

%% Plot the closed-loop response

figure('Color','w');

for i = 1:nz
    subplot(2,2,i);

    plot(t/60,Zabsolute(i,:),'LineWidth',1.5);
    hold on;
    
    plot(t/60,Zestimated(i,:),'--','LineWidth',1.2);
    
    yline(zSetpoint(i),'k:');
    
    legend('True level','Estimated level','Setpoint', ...
           'Location','best');
    grid on;
end

for i = 1:nu
    subplot(2,2,2+i);

    stairs(t(1:end-1)/60,Uabsolute(i,:),'LineWidth',1.5);
    hold on;
    yline(data.us(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('F_%d [cm^3/s]',i));
    legend('Applied flow','Nominal flow','Location','best');
    grid on;
end

sgtitle('Unconstrained MPC: linear plant and known states');

%% Plot the prediction made at the first instant

tPrediction = (1:Npred)*Ts;

figure('Color','w');

for i = 1:nz
    subplot(nz,1,i);

    plot(tPrediction/60, ...
         data.yss(i) + firstPrediction(i,:), ...
         '--','LineWidth',1.5);
    hold on;

    plot(t/60,Zabsolute(i,:),'LineWidth',1.3);
    yline(zSetpoint(i),'k:');

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('Initial prediction','Closed-loop response', ...
           'Setpoint','Location','best');

    xlim([0 Npred*Ts/60]);
    grid on;
end

%% Evaluate state estimation

rho = data.p(12);
Atank = data.p(5:8);

HallTrue = (data.xss + X) ./ (rho*Atank);
HallEstimated = (data.xss + Xestimated) ./ (rho*Atank);

figure('Color','w');

for i = 1:4
    subplot(2,2,i);

    plot(t/60,HallTrue(i,:),'k','LineWidth',1.4);
    hold on;
    plot(t/60,HallEstimated(i,:),'--','LineWidth',1.2);

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('True','Estimated','Location','best');
    grid on;
end

sgtitle('Kalman estimation during MPC control');