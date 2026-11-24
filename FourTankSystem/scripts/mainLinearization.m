clear;
clc;
close all;

%% Parameters and operating point 

p = FourTankStandardParams();

a      = p(1:4);
Atank  = p(5:8);
gamma1 = p(9);
gamma2 = p(10);
g      = p(11);
rho    = p(12);

[us,d] = FourTankOperatingInputs();

% Initial guess
xs0 = [12000;9000;5000;2500];

% Find steady-state masses
xss = fsolve(@ModifiedFourTankSystemWrap, xs0, [], us, p, d);

% Steady-state measurements
yss = FourTankSystemSensor(xss, p);
zss = FourTankSystemOutput(xss, p);

% All four equilibrium levels
hss = FourTankSystemLevels(xss,p);

%% Continuous-time linearized models

% Local tank time constants [s]
Ttank = (Atank ./ a) .* sqrt(2 * hss / g);

T1 = Ttank(1);
T2 = Ttank(2);
T3 = Ttank(3);
T4 = Ttank(4);

A = [-1/T1,  0,      1/T3,  0;
      0,    -1/T2,   0,     1/T4;
      0,     0,     -1/T3,  0;
      0,     0,      0,    -1/T4];

B = rho * [gamma1,      0;
           0,           gamma2;
           0,           1-gamma2;
           1-gamma1,    0];

E = rho * [0, 0;
           0, 0;
           1, 0;
           0, 1];

C = [1/(rho*Atank(1)), 0,                  0, 0;
     0,                  1/(rho*Atank(2)), 0, 0];

Cz = C;

disp('Tank time constants [s]:');
disp(Ttank);

disp('A ='); disp(A);
disp('B ='); disp(B);
disp('E ='); disp(E);
disp('C = Cz ='); disp(C);

disp('Eigenvalues of A [1/s]:');
disp(eig(A));


%% Continuous-time transfer functions

sysU = ss(A, B, C, zeros(2,2));
Gu = minreal(tf(sysU))

sysD = ss(A, E, C, zeros(2,2));
Gd = minreal(tf(sysD))

disp('Steady-state gain Gu(0):');
disp(dcgain(sysU));

disp('Steady-state gain Gd(0):');
disp(dcgain(sysD));


%% Compare with the model identified in Problem 4

id = load('identifiedModel.mat');
Gidentified = id.Gidentified;

% Static gains
Klin = dcgain(Gu);
Kid  = dcgain(Gidentified);

Channel = {'G11'; 'G12'; 'G21'; 'G22'};

K_linearized = zeros(4,1);
K_identified = zeros(4,1);
K_error_pct  = zeros(4,1);

% First time constant = fastest; second = slowest
Tau_fast_linearized = zeros(4,1);
Tau_fast_identified = zeros(4,1);
Tau_slow_linearized = nan(4,1);
Tau_slow_identified = nan(4,1);

row = 0;

for i = 1:2
    for j = 1:2
        row = row + 1;

        K_linearized(row) = Klin(i,j);
        K_identified(row) = Kid(i,j);

        K_error_pct(row) = 100 * (Kid(i,j) - Klin(i,j)) / Klin(i,j);

        % Extract the poles of each minimal SISO transfer function
        pLin = pole(minreal(Gu(i,j)));
        pId  = pole(minreal(Gidentified(i,j)));

        tauLin = sort(-1 ./ real(pLin));
        tauId  = sort(-1 ./ real(pId));

        Tau_fast_linearized(row) = tauLin(1);
        Tau_fast_identified(row)  = tauId(1);

        if numel(tauLin) == 2
            Tau_slow_linearized(row) = tauLin(2);
            Tau_slow_identified(row)  = tauId(2);
        end
    end
end

GainComparison = table(Channel, K_linearized, K_identified, K_error_pct)

TimeConstantComparison = table(Channel, ...
    Tau_fast_linearized, Tau_fast_identified, ...
    Tau_slow_linearized, Tau_slow_identified)


%% Discrete-time state-space model

Ts = 5;  % Sampling time [s]

sysContinuous = ss(A, [B E], C, zeros(2,4));
sysDiscrete = c2d(sysContinuous, Ts, 'zoh');

% Extract discrete-time matrices
Adisc = sysDiscrete.A;
Bdisc = sysDiscrete.B(:,1:2);
Edisc = sysDiscrete.B(:,3:4);
Cdisc = sysDiscrete.C;
Ddisc = sysDiscrete.D;

disp('Discrete-time eigenvalues:');
disp(eig(Adisc));

% Keep separate models for later use
sysUdisc = sysDiscrete(:,1:2);
sysDdisc = sysDiscrete(:,3:4);

%% Markov parameters

P4 = load('markovProblem4.mat');

Nm = P4.Nm;
k = (1:Nm)';

% Markov parameters from the linearized state-space model
Hlinearized = zeros(Nm,2,2);

% At iteration k: M = Adisc^(k-1) * Bdisc
M = Bdisc;

for n = 1:Nm
    Hlinearized(n,:,:) = reshape(Cdisc * M, [1,2,2]);
    M = Adisc * M;
end

%% Plot the comparisons

figure('Color','w');

for i = 1:2
    for j = 1:2
        subplot(2,2,(i-1)*2+j);

        plot(k, Hlinearized(:,i,j), ...
             'LineWidth',1.4);
        hold on

        plot(k, P4.Hidentified(:,i,j), '--', ...
             'LineWidth',1.2);

        plot(k, P4.Hstep(:,i,j), ':', ...
             'LineWidth',1.4);

        grid on
        xlabel('$k$', 'Interpreter','latex');
        ylabel(sprintf('$[H_{k}]_{%d%d}$',i,j), ...
               'Interpreter','latex');

        legend('Linearized model', ...
               'Identified model', ...
               'Nonlinear step differences', ...
               'Location','best');
    end
end

sgtitle(sprintf('Markov parameter comparison, T_s = %g s',Ts));


%% Model 3: Langevin disturbances

% Same parameters as GenerateDisturbances, case 'langevin'
aF = [0.01; 0.01];        % Mean-reversion rates [1/s]
sigmaF = [0.2; 0.2];      % Diffusion coefficients

xss3 = [xss; d];

%% Continuous-time linearization

% "3" distinguishes model 3 from the four-state model
A3 = [A,          E;
      zeros(2,4), -diag(aF)];

B3 = [B;
      zeros(2,2)];

C3 = [C, zeros(2,2)];
Cz3 = C3;

Sigma3 = [zeros(4,2);
          diag(sigmaF)];

disp('Model 3: continuous-time eigenvalues');
disp(eig(A3));

%% Continuous-time transfer function

sysU3 = ss(A3, B3, C3, zeros(2,2));
Gu3 = minreal(tf(sysU3))

%% Discretization of the deterministic drift and commands
[A3disc, B3disc, Q3] = c2dzohsw(A3,B3,Sigma3,Ts)
C3disc = C3;

%{
sysU3disc = c2d(sysU3, Ts, 'zoh');

A3disc = sysU3disc.A;
B3disc = sysU3disc.B;
C3disc = sysU3disc.C;

disp('Model 3: discrete-time eigenvalues');
disp(eig(A3disc));

% Discrete process-noise covariance for model 3
nx3 = size(A3,1);

V = expm([-A3,          Sigma3*Sigma3';
          zeros(nx3),  A3'] * Ts);

Q3 = A3disc * V(1:nx3,nx3+1:2*nx3);
Q3 = (Q3 + Q3') / 2;  % Remove numerical asymmetry
%}
%% Command-to-output Markov parameters

H3 = zeros(Nm,2,2);
M3 = B3disc;

for n = 1:Nm
    H3(n,:,:) = reshape(C3disc * M3, [1,2,2]);
    M3 = A3disc * M3;
end

% These coefficients should match those of model 1
differenceH = H3 - Hlinearized;

fprintf('Maximum Markov difference: model 3 vs model 1 = %.3e\n', ...
        max(abs(differenceH(:))));


%% Save the linear model

save('FourTankSystem/data/linearModel.mat', 'Adisc', 'Bdisc', 'Edisc', 'Cdisc', 'Ts', 'xss', 'yss', 'us', 'd', 'p');
save('FourTankSystem/data/linearModel3.mat', 'A3disc','B3disc','C3disc','Q3', 'Ts','xss3','yss','us','d','p');