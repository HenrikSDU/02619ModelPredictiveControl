clear;
clc;
close all;

%% Load the physical linearized model

data = load('linearModel.mat');

A = data.Adisc;
B = data.Bdisc;
E = data.Edisc;
C = data.Cdisc;
Ts = data.Ts;

nx = size(A,1);
nu = size(B,2);
ny = size(C,1);
nd = size(E,2);

%% Simulation settings

t = (0:Ts:30*60)';
N = numel(t);

rng(1);

% Constant nominal pump flows: zero input deviations
U = zeros(nu,N);

% Unknown changes in mean disturbance flows [cm^3/s]
Dstep = zeros(nd,N);

Dstep(1,t >= 5*60) = 10;
Dstep(2,t >= 12*60) = -5;

%% Actual noise and measurement covariances

sigmaProcess = 5;       % Flow fluctuations [cm^3/s]
sigmaMeasurement = 0.1; % Measurement noise [cm]

Q = sigmaProcess^2 * eye(nd);
R = sigmaMeasurement^2 * eye(ny);

LQ = chol(Q,'lower');
LR = chol(R,'lower');

%% Filters without disturbance augmentation

G = E;

[Kstatic,~,~] = DesignStationaryKalmanFilter(A,G,C,Q,R);

%% Filters with random-walk disturbance states

Aa = [A,           E;
      zeros(nd,nx), eye(nd)];

Ba = [B;
      zeros(nd,nu)];

Ca = [C, zeros(ny,nd)];

% Independent noises:
% w_k acts on the tank masses through E
% eta_k acts on the disturbance-bias states
Ga = [E,            zeros(nx,nd);
      zeros(nd,nd), eye(nd)];

% Random-walk standard deviation per sample [cm^3/s]
% Tuning parameter: larger values allow faster bias adaptation.
sigmaBias = 0.2;
Qbias = sigmaBias^2 * eye(nd);

Qa = blkdiag(Q,Qbias);

nxa = size(Aa,1);

%% Check detectability of the added integrating modes

% The original A is stable. The new modes are at lambda = 1.
detectabilityMatrix = [eye(nx)-A, -E;
                       C,         zeros(ny,nd)];

assert(rank(detectabilityMatrix) == nxa, ...
       'The augmented integrating modes are not observable.');

[KstaticAug,~,~] = ...
    DesignStationaryKalmanFilter(Aa,Ga,Ca,Qa,R);

%% Generate one plant trajectory and one measurement sequence

X = zeros(nx,N);
Y = zeros(ny,N);

% Plant starts at nominal equilibrium
X(:,1) = zeros(nx,1);

for k = 1:N

    Y(:,k) = C*X(:,k) + LR*randn(ny,1);

    if k < N
        wk = LQ*randn(nd,1);

        X(:,k+1) = A*X(:,k) + B*U(:,k) ...
                    + E*(Dstep(:,k) + wk);
    end
end

% Dstep is used only by the simulated plant.
% It is never supplied to any filter.

%% Initialize the four filters

Xstatic = zeros(nx,N);
Xdynamic = zeros(nx,N);

XstaticAug = zeros(nxa,N);
XdynamicAug = zeros(nxa,N);

xPredStatic = zeros(nx,1);
xPredDynamic = zeros(nx,1);

xPredStaticAug = zeros(nxa,1);
xPredDynamicAug = zeros(nxa,1);

% Initial equilibrium is assumed known
PpredDynamic = zeros(nx);
PpredDynamicAug = zeros(nxa);

%% Apply all filters to the same data

for k = 1:N

    % Standard stationary filter
    [Xstatic(:,k), xNextStatic] = ...
        KalmanFilterStationary( ...
            xPredStatic,Y(:,k),U(:,k),A,B,C,Kstatic);

    % Standard dynamic filter
    [Xdynamic(:,k), xNextDynamic, PnextDynamic] = ...
        KalmanFilterDynamic( ...
            xPredDynamic,PpredDynamic,Y(:,k),U(:,k), ...
            A,B,G,C,Q,R);

    % Augmented stationary filter
    [XstaticAug(:,k), xNextStaticAug] = ...
        KalmanFilterStationary( ...
            xPredStaticAug,Y(:,k),U(:,k), ...
            Aa,Ba,Ca,KstaticAug);

    % Augmented dynamic filter
    [XdynamicAug(:,k), xNextDynamicAug, PnextDynamicAug] = ...
        KalmanFilterDynamic( ...
            xPredDynamicAug,PpredDynamicAug,Y(:,k),U(:,k), ...
            Aa,Ba,Ga,Ca,Qa,R);

    if k < N
        xPredStatic = xNextStatic;

        xPredDynamic = xNextDynamic;
        PpredDynamic = PnextDynamic;

        xPredStaticAug = xNextStaticAug;

        xPredDynamicAug = xNextDynamicAug;
        PpredDynamicAug = PnextDynamicAug;
    end
end

%% Convert mass deviations into absolute tank levels

rho = data.p(12);
Atank = data.p(5:8);
massPerCm = rho*Atank;

Htrue = (data.xss + X) ./ massPerCm;

Hstatic = (data.xss + Xstatic) ./ massPerCm;
Hdynamic = (data.xss + Xdynamic) ./ massPerCm;

HstaticAug = (data.xss + XstaticAug(1:nx,:)) ./ massPerCm;
HdynamicAug = (data.xss + XdynamicAug(1:nx,:)) ./ massPerCm;

%% Plot all four tank levels

figure('Color','w');

for i = 1:nx
    subplot(2,2,i);

    plot(t/60,Htrue(i,:),'k','LineWidth',1.5);
    hold on;
    plot(t/60,Hstatic(i,:),'--','LineWidth',1.1);
    plot(t/60,Hdynamic(i,:),':','LineWidth',1.1);
    plot(t/60,HstaticAug(i,:),'-.','LineWidth',1.3);
    plot(t/60,HdynamicAug(i,:),'LineWidth',1.1);

    xlabel('Time [min]');
    ylabel(sprintf('h_%d [cm]',i));
    legend('True','Standard stationary','Standard dynamic', ...
           'Augmented stationary','Augmented dynamic', ...
           'Location','best');
    grid on;
end

sgtitle('Unknown disturbance steps: state estimation');

%% Plot the estimated changes in mean disturbance

Bstatic = XstaticAug(nx+1:end,:);
Bdynamic = XdynamicAug(nx+1:end,:);

figure('Color','w');

for i = 1:nd
    subplot(nd,1,i);

    stairs(t/60,Dstep(i,:),'k','LineWidth',1.5);
    hold on;
    plot(t/60,Bstatic(i,:),'--','LineWidth',1.3);
    plot(t/60,Bdynamic(i,:),':','LineWidth',1.3);

    xlabel('Time [min]');
    ylabel(sprintf('Bias F_%d [cm^3/s]',i+2));
    legend('True mean change','Augmented stationary', ...
           'Augmented dynamic','Location','best');
    grid on;
end

sgtitle('Estimation of the unknown disturbance mean changes');

%% Compare errors after the final transient

% Final 5 minutes, well after both imposed steps
evaluationWindow = t >= 25*60;

estimates = {Hstatic,Hdynamic,HstaticAug,HdynamicAug};

names = {'Standard stationary'; ...
         'Standard dynamic'; ...
         'Augmented stationary'; ...
         'Augmented dynamic'};

Bias = zeros(nx,4);
RMSE = zeros(nx,4);

for j = 1:4
    err = estimates{j}(:,evaluationWindow) ...
          - Htrue(:,evaluationWindow);

    Bias(:,j) = mean(err,2);
    RMSE(:,j) = sqrt(mean(err.^2,2));
end

Tank = (1:nx)';

BiasComparison = table(Tank, ...
    Bias(:,1),Bias(:,2),Bias(:,3),Bias(:,4), ...
    'VariableNames',{'Tank','StandardStatic','StandardDynamic', ...
                    'AugmentedStatic','AugmentedDynamic'})

RMSEComparison = table(Tank, ...
    RMSE(:,1),RMSE(:,2),RMSE(:,3),RMSE(:,4), ...
    'VariableNames',{'Tank','StandardStatic','StandardDynamic', ...
                    'AugmentedStatic','AugmentedDynamic'})

disp('Bias and RMSE are expressed in cm.');