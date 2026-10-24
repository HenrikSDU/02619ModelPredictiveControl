clear;
clc;
close all;

%% Load the discrete-time linear model

data = load('linearModel.mat');

A = data.Adisc;
B = data.Bdisc;
G = data.Edisc;
C = data.Cdisc;
Ts = data.Ts;

nx = size(A,1);
nu = size(B,2);
ny = size(C,1);
nw = size(G,2);

%% Noise covariances

sigmaProcess = 5;        % Flow standard deviation [cm^3/s]
sigmaMeasurement = 0.1;  % Level standard deviation [cm]

Q = sigmaProcess^2 * eye(nw);
R = sigmaMeasurement^2 * eye(ny);

LQ = chol(Q,'lower');
LR = chol(R,'lower');

%% Design the stationary filter

[KfxStatic,Pstationary,ReStationary] = DesignStationaryKalmanFilter(A,G,C,Q,R);

%% Simulation settings

t = (0:Ts:20*60)';
N = numel(t);

rng(1);

U = zeros(nu,N);

X = zeros(nx,N);
Y = zeros(ny,N);

XfilteredStatic = zeros(nx,N);
XfilteredDynamic = zeros(nx,N);

GainDynamic = zeros(nx,ny,N);

% Same initial estimated mean for both filters
xPredictedStatic = zeros(nx,1);
xPredictedDynamic = zeros(nx,1);

% Initial uncertainty: standard deviation of 1 cm in each tank
rho = data.p(12);
Atank = data.p(5:8);

sigmaH0 = 1;  % [cm]
sigmaMass0 = rho * Atank * sigmaH0;  % [g]

P0 = diag(sigmaMass0.^2);
PpredictedDynamic = P0;

% Simulated initial state deviation, consistent with P0
X(:,1) = chol(P0,'lower') * randn(nx,1);

%% Simulation and estimation

for k = 1:N

    % Simulated measurement: shared by both filters
    vk = LR * randn(ny,1);
    Y(:,k) = C*X(:,k) + vk;

    % Stationary filter
    [XfilteredStatic(:,k), xNextStatic] = ...
        KalmanFilterStationary( ...
            xPredictedStatic, Y(:,k), U(:,k), ...
            A, B, C, KfxStatic);

    % Dynamic filter
    [XfilteredDynamic(:,k), xNextDynamic, PnextDynamic, ~, Kk] = ...
        KalmanFilterDynamic( ...
            xPredictedDynamic, PpredictedDynamic, ...
            Y(:,k), U(:,k), A, B, G, C, Q, R);

    GainDynamic(:,:,k) = Kk;

    if k < N

        % Simulated plant: same trajectory for both filters
        wk = LQ * randn(nw,1);
        X(:,k+1) = A*X(:,k) + B*U(:,k) + G*wk;

        % Prepare the filters for the next measurement
        xPredictedStatic = xNextStatic;

        xPredictedDynamic = xNextDynamic;
        PpredictedDynamic = PnextDynamic;
    end
end

%% Convert state deviations into absolute tank levels

HallTrue = (data.xss + X) ./ (rho*Atank);

HallStatic = (data.xss + XfilteredStatic) ./ (rho*Atank);
HallDynamic = (data.xss + XfilteredDynamic) ./ (rho*Atank);

figure('Color','w');

for i = 1:nx
    subplot(2,2,i);

    plot(t/60,HallTrue(i,:),'k','LineWidth',1.4);
    hold on;
    plot(t/60,HallStatic(i,:),'--','LineWidth',1.2);
    plot(t/60,HallDynamic(i,:),':','LineWidth',1.4);

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('True','Stationary KF','Dynamic KF', ...
           'Location','best');
    grid on;
end

sgtitle('Stationary and dynamic Kalman filters');

%% Compare estimation errors

errorStatic = HallStatic - HallTrue;
errorDynamic = HallDynamic - HallTrue;

RMSE_static = sqrt(mean(errorStatic.^2,2));
RMSE_dynamic = sqrt(mean(errorDynamic.^2,2));

Tank = (1:nx)';

ErrorComparison = table(Tank, RMSE_static, RMSE_dynamic)


%% Langevin model

%% Load the discrete-time linear model

data = load('linearModel3.mat');

A = data.A3disc;
B = data.B3disc;
C = data.C3disc;

% Q3 already includes the effect of the continuous diffusion
G = eye(6);
Q = data.Q3;

sigmaMeasurement = 0.1;
R = sigmaMeasurement^2 * eye(2);

Ts = data.Ts;

nx = 6;
nu = 2;
ny = 2;
nw = 6;

LR = chol(R,'lower');

% Covariance square root, also valid for semidefinite Q
[Vq,Dq] = eig((Q+Q')/2);
LQ = Vq * diag(sqrt(max(diag(Dq),0)));

%% Design the stationary filter

[KfxStatic,Pstationary,ReStationary] = DesignStationaryKalmanFilter(A,G,C,Q,R);

%% Simulation settings

t = (0:Ts:20*60)';
N = numel(t);

rng(1);

U = zeros(nu,N);

X = zeros(nx,N);
Y = zeros(ny,N);

XfilteredStatic = zeros(nx,N);
XfilteredDynamic = zeros(nx,N);

GainDynamic = zeros(nx,ny,N);

% Same initial estimated mean for both filters
xPredictedStatic = zeros(nx,1);
xPredictedDynamic = zeros(nx,1);

% Initial uncertainty: standard deviation of 1 cm in each tank
rho = data.p(12);
Atank = data.p(5:8);

sigmaH0 = 1;  % [cm]
sigmaMass0 = rho * Atank * sigmaH0;  % [g]

sigmaFlow0 = [2;2]; % Initial flow uncertainty [cm^3/s]

P0 = diag([sigmaMass0.^2; sigmaFlow0.^2]);
PpredictedDynamic = P0;

X(:,1) = chol(P0,'lower') * randn(nx,1);

%% Simulation and estimation

for k = 1:N

    % Simulated measurement: shared by both filters
    vk = LR * randn(ny,1);
    Y(:,k) = C*X(:,k) + vk;

    % Stationary filter
    [XfilteredStatic(:,k), xNextStatic] = ...
        KalmanFilterStationary( ...
            xPredictedStatic, Y(:,k), U(:,k), ...
            A, B, C, KfxStatic);

    % Dynamic filter
    [XfilteredDynamic(:,k), xNextDynamic, PnextDynamic, ~, Kk] = ...
        KalmanFilterDynamic( ...
            xPredictedDynamic, PpredictedDynamic, ...
            Y(:,k), U(:,k), A, B, G, C, Q, R);

    GainDynamic(:,:,k) = Kk;

    if k < N

        % Simulated plant: same trajectory for both filters
        wk = LQ * randn(nw,1);
        X(:,k+1) = A*X(:,k) + B*U(:,k) + G*wk;

        % Prepare the filters for the next measurement
        xPredictedStatic = xNextStatic;

        xPredictedDynamic = xNextDynamic;
        PpredictedDynamic = PnextDynamic;
    end
end

%% Absolute tank levels: states 1 to 4

HallTrue = (data.xss3(1:4) + X(1:4,:)) ./ (rho*Atank);

HallStatic = (data.xss3(1:4) + XfilteredStatic(1:4,:)) ...
             ./ (rho*Atank);

HallDynamic = (data.xss3(1:4) + XfilteredDynamic(1:4,:)) ...
              ./ (rho*Atank);

figure('Color','w');

for i = 1:4
    subplot(2,2,i);

    plot(t/60,HallTrue(i,:),'k','LineWidth',1.4);
    hold on;
    plot(t/60,HallStatic(i,:),'--','LineWidth',1.2);
    plot(t/60,HallDynamic(i,:),':','LineWidth',1.4);

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('True','Stationary KF','Dynamic KF', ...
           'Location','best');
    grid on;
end

sgtitle('Langevin model: tank-level estimation');

%% Absolute disturbances: states 5 and 6

Dtrue = data.xss3(5:6) + X(5:6,:);
Dstatic = data.xss3(5:6) + XfilteredStatic(5:6,:);
Ddynamic = data.xss3(5:6) + XfilteredDynamic(5:6,:);

figure('Color','w');

for i = 1:2
    subplot(2,1,i);

    plot(t/60,Dtrue(i,:),'k','LineWidth',1.4);
    hold on;
    plot(t/60,Dstatic(i,:),'--','LineWidth',1.2);
    plot(t/60,Ddynamic(i,:),':','LineWidth',1.4);

    xlabel('Time [min]');
    ylabel(sprintf('F_%d [cm^3/s]',i+2));
    legend('True','Stationary KF','Dynamic KF', ...
           'Location','best');
    grid on;
end

sgtitle('Langevin model: disturbance estimation');

%% Level estimation errors [cm]

RMSE_static = sqrt(mean((HallStatic-HallTrue).^2,2));
RMSE_dynamic = sqrt(mean((HallDynamic-HallTrue).^2,2));

Tank = (1:4)';
LevelErrorComparison = table(Tank, RMSE_static, RMSE_dynamic)

%% Disturbance estimation errors [cm^3/s]

RMSE_static = sqrt(mean((Dstatic-Dtrue).^2,2));
RMSE_dynamic = sqrt(mean((Ddynamic-Dtrue).^2,2));

Disturbance = {'F3';'F4'};
DisturbanceErrorComparison = table( ...
    Disturbance, RMSE_static, RMSE_dynamic)