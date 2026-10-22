clear;
clc;
close all;

%% Parameters

p = FourTankStandardParams();

F3 = 50;
F4 = 30;

d = [F3; F4];

%% Steady-state operating point

F1 = 250; 
F2 = 325;

us = [F1; F2];

% Initial guess
xs0 = [12000;9000;5000;2500];

% Find steady-state masses
xs = fsolve(@ModifiedFourTankSystemWrap, xs0, [], us, p, d);

% Steady-state measurements
ys = FourTankSystemSensor(xs, p);
zs = FourTankSystemOutput(xs, p);

fprintf('Steady-state levels:\n');
disp(ys);

fprintf('Steady-state masses:\n');
disp(xs);

%% Simulation settings

tend = 20*60;     % [s]

stepSizes = [0.10 0.25 0.50];

%% Simulate steps in F1

[T10_F1, X10_F1, H10_F1] = simulateStep(us, xs, p, 0.10, 1, tend, d);

[T25_F1, X25_F1, H25_F1] = simulateStep(us, xs, p, 0.25, 1, tend, d);

[T50_F1, X50_F1, H50_F1] = simulateStep(us, xs, p, 0.50, 1, tend, d);


% Plots

figure("Color","w");

for i = 1:4

    subplot(2,2,i)

    plot(T10_F1/60, H10_F1(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F1/60, H25_F1(:,i), 'LineWidth', 1.2)
    plot(T50_F1/60, H50_F1(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('h_%d [cm]', i))

    legend('10%', '25%', '50%')
    grid on

    if i==3
        ylim([6 9]) % abs()
    end

end

sgtitle('Nonlinear step responses F_1')

% Initial steady-state levels
h0 = ys(:)';

% Input step magnitudes
dF1_10 = 0.10 * us(1);
dF1_25 = 0.25 * us(1);
dF1_50 = 0.50 * us(1);

% Normalize responses
S10_F1 = (H10_F1 - h0) / dF1_10;
S25_F1 = (H25_F1 - h0) / dF1_25;
S50_F1 = (H50_F1 - h0) / dF1_50;

% Plot normalized responses
figure("Color","w");

for i = 1:4

    subplot(2,2,i)

    plot(T10_F1/60, S10_F1(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F1/60, S25_F1(:,i), 'LineWidth', 1.2)
    plot(T50_F1/60, S50_F1(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('Normalized response S_{%d1}',i))

    legend('10% step','25% step','50% step', ...
           'Location','best')

    if i==3
        ylim([-1 1]) % abs()
    end

    grid on

end

sgtitle('Normalized nonlinear step responses F_1')

%% Simulate steps in F2

[T10_F2, X10_F2, H10_F2] = simulateStep(us, xs, p, 0.10, 2, tend, d);

[T25_F2, X25_F2, H25_F2] = simulateStep(us, xs, p, 0.25, 2, tend, d);

[T50_F2, X50_F2, H50_F2] = simulateStep(us, xs, p, 0.50, 2, tend, d);

% PLots

figure("Color","w");

for i = 1:4

    subplot(2,2,i)
    

    plot(T10_F2/60, H10_F2(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F2/60, H25_F2(:,i), 'LineWidth', 1.2)
    plot(T50_F2/60, H50_F2(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('h_%d [cm]', i))

    legend('10%', '25%', '50%')

    grid on

    if i==4
        ylim([5 6.5]) % abs()
    end

end

sgtitle('Nonlinear step responses F_2')

% Initial steady-state levels
h0 = ys(:)';

% Input step magnitudes
dF2_10 = 0.10 * us(2);
dF2_25 = 0.25 * us(2);
dF2_50 = 0.50 * us(2);

% Normalize responses
S10_F2 = (H10_F2 - h0) / dF2_10;
S25_F2 = (H25_F2 - h0) / dF2_25;
S50_F2 = (H50_F2 - h0) / dF2_50;

% Plot normalized responses
figure("Color","w");

for i = 1:4

    subplot(2,2,i)

    plot(T10_F2/60, S10_F2(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F2/60, S25_F2(:,i), 'LineWidth', 1.2)
    plot(T50_F2/60, S50_F2(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('Normalized response S_{%d2}',i))

    legend('10% step','25% step','50% step', ...
           'Location','best')

    if i==4
        ylim([-1 1]) % abs()
    end

    grid on

end

sgtitle('Normalized nonlinear step responses F_2')


%% With process and measurement noise

dMean = [F3; F4];

Ts = 10;

%% Noise levels

% Standard deviation of process noise [cm^3/s]
sigmaProcess_low    = 1;
sigmaProcess_medium = 5;
sigmaProcess_high   = 10;

% Standard deviation of measurement noise [cm]
sigmaMeasurement_low    = 0.02;
sigmaMeasurement_medium = 0.10;
sigmaMeasurement_high   = 0.30;

rng(1);

% Initial steady-state output
h0 = ys(:)';

% Actual input step magnitudes
dF1_10 = 0.10 * us(1);
dF1_25 = 0.25 * us(1);
dF1_50 = 0.50 * us(1);

%% Simulate steps in F1

noiseNames = {'Low noise','Medium noise','High noise'};

sigmaProcess = [sigmaProcess_low ...
                sigmaProcess_medium ...
                sigmaProcess_high];

sigmaMeasurement = [sigmaMeasurement_low ...
                    sigmaMeasurement_medium ...
                    sigmaMeasurement_high];

for n = 1:3

    % Simulate 10% step

    [T10,X10,H10,Y10,D10] = simulateStepNoise( ...
        us, xs, p, ...
        0.10, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 25% step

    [T25,X25,H25,Y25,D25] = simulateStepNoise( ...
        us, xs, p, ...
        0.25, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 50% step

    [T50,X50,H50,Y50,D50] = simulateStepNoise( ...
        us, xs, p, ...
        0.50, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Plot measured responses

    figure("Color","w");

    for i = 1:4

        subplot(2,2,i)

        plot(T10/60,Y10(:,i),'LineWidth',1.0)
        hold on

        plot(T25/60,Y25(:,i),'LineWidth',1.0)
        plot(T50/60,Y50(:,i),'LineWidth',1.0)

        xlabel('Time [min]')
        ylabel(sprintf('y_%d [cm]',i))

        legend('10%','25%','50%','Location','best')

        if i==3
            ylim([5 10]) % abs()
        end

        grid on

    end

    sgtitle(sprintf('F_1 step responses - %s',noiseNames{n}));

    % Normalize measured responses
    S10 = (Y10 - h0) / dF1_10;
    S25 = (Y25 - h0) / dF1_25;
    S50 = (Y50 - h0) / dF1_50;

    % Plot normalized responses
    figure("Color","w");

    for i = 1:4

        subplot(2,2,i)

        plot(T10/60, S10(:,i), 'LineWidth', 1.0)
        hold on

        plot(T25/60, S25(:,i), 'LineWidth', 1.0)
        plot(T50/60, S50(:,i), 'LineWidth', 1.0)

        xlabel('Time [min]')
        ylabel(sprintf('Normalized response S_{%d1}',i))

        legend('10% step','25% step','50% step', ...
               'Location','best')

        if i==3
            ylim([-1 1]) % abs()
        end

        grid on

    end

    sgtitle(sprintf('Normalized F_1 responses - %s', ...
                    noiseNames{n}));

end


%% Simulate steps in F2

% Initial steady-state output
h0 = ys(:)';

% Actual input step magnitudes
dF2_10 = 0.10 * us(2);
dF2_25 = 0.25 * us(2);
dF2_50 = 0.50 * us(2);

for n = 1:3

    % Simulate 10% step

    [T10,X10,H10,Y10,D10] = simulateStepNoise( ...
        us, xs, p, ...
        0.10, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 25% step

    [T25,X25,H25,Y25,D25] = simulateStepNoise( ...
        us, xs, p, ...
        0.25, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 50% step

    [T50,X50,H50,Y50,D50] = simulateStepNoise( ...
        us, xs, p, ...
        0.50, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Plot measured responses

    figure("Color","w");

    for i = 1:4

        subplot(2,2,i)

        plot(T10/60,Y10(:,i),'LineWidth',1.0)
        hold on

        plot(T25/60,Y25(:,i),'LineWidth',1.0)
        plot(T50/60,Y50(:,i),'LineWidth',1.0)

        xlabel('Time [min]')
        ylabel(sprintf('y_%d [cm]',i))

        legend('10%','25%','50%','Location','best')

        if i==4
            ylim([4 7]) % abs()
        end

        grid on

    end

    sgtitle(sprintf('F_2 step responses - %s',noiseNames{n}));


    % Normalize measured responses
    S10 = (Y10 - h0) / dF2_10;
    S25 = (Y25 - h0) / dF2_25;
    S50 = (Y50 - h0) / dF2_50;

    % Plot normalized responses
    figure("Color","w");

    for i = 1:4

        subplot(2,2,i)

        plot(T10/60, S10(:,i), 'LineWidth', 1.0)
        hold on

        plot(T25/60, S25(:,i), 'LineWidth', 1.0)
        plot(T50/60, S50(:,i), 'LineWidth', 1.0)

        xlabel('Time [min]')
        ylabel(sprintf('Normalized response S_{%d2}',i))

        legend('10% step','25% step','50% step', ...
               'Location','best')

        if i==4
            ylim([-1 1]) % abs()
        end

        grid on

    end

    sgtitle(sprintf('Normalized F_2 responses - %s', ...
                    noiseNames{n}));

end

%% Q4: Identification from normalized steps

close all;

Tsid = 5;                              % [s] uniform sampling
tu   = (0:Tsid:tend)';
Nu   = numel(tu);

S1 = interp1(T10_F1, S10_F1, tu);      % N x 4  (response to F1)
S2 = interp1(T10_F2, S10_F2, tu);      % N x 4  (response to F2)
Sid = {S1, S2};

np = [1 2;      % G11 G12
      2 1;      % G21 G22
      0 1;      % G31 = 0, G32
      1 0];     % G41, G42 = 0

opt = tfestOptions('InitialCondition','zero');
G = tf(zeros(4,2));
fitPct = nan(4,2);

for j = 1:2
    for i = 1:4
        if np(i,j) == 0, continue; end
        data  = iddata(Sid{j}(:,i), ones(Nu,1), Tsid);   % unit step input
        Gij   = tfest(data, np(i,j), 0, opt);
        G(i,j) = tf(Gij);
        fitPct(i,j) = goodnessOfFit(lsim(Gij,ones(Nu,1),tu), Sid{j}(:,i),'NRMSE')*100;
    end
end

disp('Identified G(s) [cm/(cm^3/s)]:'); G
disp('Fit [%] per channel:'); disp(fitPct)

%% Validation against the 25% and 50% nonlinear steps

% Each dataset keeps its own time vector
Tcell = {T25_F1, T50_F1; T25_F2, T50_F2};
Scell = {S25_F1, S50_F1; S25_F2, S50_F2};
amp   = {'25%','50%'};

for j = 1:2
    figure('Color','w');
    for i = 1:4
        subplot(2,2,i); hold on; grid on

        for a = 1:2
            Sn = interp1(Tcell{j,a}, Scell{j,a}(:,i), tu, 'linear', 'extrap');
            plot(tu/60, Sn, 'LineWidth', 1.1, ...
                 'DisplayName', [amp{a} ' nonlinear']);
        end

        if np(i,j) > 0
            plot(tu/60, step(G(i,j), tu), '--k', ...
                 'LineWidth', 1.2, 'DisplayName', 'Identified model');
        end

        xlabel('Time [min]');
        ylabel(sprintf('S_{%d%d}', i, j));
        legend('Location','best');
    end
    sgtitle(sprintf('Identified model vs nonlinear, input F_%d', j));
end

%% Markov Parameters

Tm = 5;  Nm = 60;                       % sampling time [s], number of coefficients
Gd = c2d(G, Tm, 'zoh');
[h, tk] = impulse(Gd, (Nm-1)*Tm);       % h(k, out, in), tk in seconds
k = round(tk/Tm);

figure('Color','w');
for j = 1:2
    for i = 1:4
        subplot(4,2,(i-1)*2+j);
        stem(k, h(:,i,j), 'filled', 'MarkerSize',3); grid on
        xlabel('k'); ylabel(sprintf('h_{%d%d}[k]',i,j));
    end
end
sgtitle(sprintf('Markov parameters, T_s = %d s',Tm));

% Check directly from the normalized step data: h[k] = S[k] - S[k-1]
Sm = interp1(T10_F1, S10_F1, (0:Tm:(Nm-1)*Tm)');
hdata = diff([0; Sm(:,1)]);             % compare with h(:,1,1) shifted by one sample