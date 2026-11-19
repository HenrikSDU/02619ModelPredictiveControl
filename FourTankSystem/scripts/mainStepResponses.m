clear;
clc;
close all;

%% Parameters

p = FourTankStandardParams();

%% Steady-state operating point

[us,d] = FourTankOperatingInputs();

% Initial guess
xs0 = [12000;9000;5000;2500];

% Find steady-state masses
xss = fsolve(@ModifiedFourTankSystemWrap, xs0, [], us, p, d);

hss = FourTankSystemLevels(xss,p);

% Steady-state measurements
yss = FourTankSystemSensor(xss, p);
zss = FourTankSystemOutput(xss, p);

fprintf('Steady-state levels:\n');
disp(yss);

fprintf('Steady-state masses:\n');
disp(xss);

%% Simulation settings

tend = 20*60;     % [s]

stepSizes = [0.10 0.25 0.50];

%% Simulate steps in F1

[T10_F1, X10_F1, H10_F1] = simulateStep(us, xss, p, 0.10, 1, tend, d);

[T25_F1, X25_F1, H25_F1] = simulateStep(us, xss, p, 0.25, 1, tend, d);

[T50_F1, X50_F1, H50_F1] = simulateStep(us, xss, p, 0.50, 1, tend, d);


% Plots

figure("Color","w");

for i = 1:2

    subplot(2,1,i)

    plot(T10_F1/60, H10_F1(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F1/60, H25_F1(:,i), 'LineWidth', 1.2)
    plot(T50_F1/60, H50_F1(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('h_%d [cm]', i))

    legend('10%', '25%', '50%')
    grid on

end

sgtitle('Nonlinear step responses F_1')

% Initial steady-state levels
h0 = hss(:)';

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

for i = 1:2

    subplot(2,1,i)

    plot(T10_F1/60, S10_F1(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F1/60, S25_F1(:,i), 'LineWidth', 1.2)
    plot(T50_F1/60, S50_F1(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('Normalized response S_{%d1}',i))

    legend('10% step','25% step','50% step', ...
           'Location','best')

    grid on

end

sgtitle('Normalized nonlinear step responses F_1')

%% Simulate steps in F2

[T10_F2, X10_F2, H10_F2] = simulateStep(us, xss, p, 0.10, 2, tend, d);

[T25_F2, X25_F2, H25_F2] = simulateStep(us, xss, p, 0.25, 2, tend, d);

[T50_F2, X50_F2, H50_F2] = simulateStep(us, xss, p, 0.50, 2, tend, d);

% PLots

figure("Color","w");

for i = 1:2

    subplot(2,1,i)
    

    plot(T10_F2/60, H10_F2(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F2/60, H25_F2(:,i), 'LineWidth', 1.2)
    plot(T50_F2/60, H50_F2(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('h_%d [cm]', i))

    legend('10%', '25%', '50%')

    grid on

end

sgtitle('Nonlinear step responses F_2')

% Initial steady-state levels
h0 = hss(:)';

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

for i = 1:2

    subplot(2,1,i)

    plot(T10_F2/60, S10_F2(:,i), 'LineWidth', 1.2)
    hold on

    plot(T25_F2/60, S25_F2(:,i), 'LineWidth', 1.2)
    plot(T50_F2/60, S50_F2(:,i), 'LineWidth', 1.2)

    xlabel('Time [min]')
    ylabel(sprintf('Normalized response S_{%d2}',i))

    legend('10% step','25% step','50% step', ...
           'Location','best')

    grid on

end

sgtitle('Normalized nonlinear step responses F_2')


%% With process and measurement noise

dMean = d;

Ts = 5;

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
y0 = yss(:)';

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
        us, xss, p, ...
        0.10, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 25% step

    [T25,X25,H25,Y25,D25] = simulateStepNoise( ...
        us, xss, p, ...
        0.25, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 50% step

    [T50,X50,H50,Y50,D50] = simulateStepNoise( ...
        us, xss, p, ...
        0.50, 1, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Plot measured responses

    figure("Color","w");

    for i = 1:2

        subplot(2,1,i)

        plot(T10/60,Y10(:,i),'LineWidth',1.0)
        hold on

        plot(T25/60,Y25(:,i),'LineWidth',1.0)
        plot(T50/60,Y50(:,i),'LineWidth',1.0)

        xlabel('Time [min]')
        ylabel(sprintf('y_%d [cm]',i))

        legend('10%','25%','50%','Location','best')

        grid on

    end

    sgtitle(sprintf('F_1 step responses - %s',noiseNames{n}));

    % Normalize measured responses
    S10 = (Y10 - y0) / dF1_10;
    S25 = (Y25 - y0) / dF1_25;
    S50 = (Y50 - y0) / dF1_50;

    % Plot normalized responses
    figure("Color","w");

    for i = 1:2

        subplot(2,1,i)

        plot(T10/60, S10(:,i), 'LineWidth', 1.0)
        hold on

        plot(T25/60, S25(:,i), 'LineWidth', 1.0)
        plot(T50/60, S50(:,i), 'LineWidth', 1.0)

        xlabel('Time [min]')
        ylabel(sprintf('Normalized response S_{%d1}',i))

        legend('10% step','25% step','50% step', ...
               'Location','best')

        grid on

    end

    sgtitle(sprintf('Normalized F_1 responses - %s', ...
                    noiseNames{n}));

end


%% Simulate steps in F2

% Initial steady-state output
y0 = yss(:)';

% Actual input step magnitudes
dF2_10 = 0.10 * us(2);
dF2_25 = 0.25 * us(2);
dF2_50 = 0.50 * us(2);

for n = 1:3

    % Simulate 10% step

    [T10,X10,H10,Y10,D10] = simulateStepNoise( ...
        us, xss, p, ...
        0.10, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 25% step

    [T25,X25,H25,Y25,D25] = simulateStepNoise( ...
        us, xss, p, ...
        0.25, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Simulate 50% step

    [T50,X50,H50,Y50,D50] = simulateStepNoise( ...
        us, xss, p, ...
        0.50, 2, tend, dMean, Ts, ...
        sigmaProcess(n), sigmaMeasurement(n));

    % Plot measured responses

    figure("Color","w");

    for i = 1:2

        subplot(2,1,i)

        plot(T10/60,Y10(:,i),'LineWidth',1.0)
        hold on

        plot(T25/60,Y25(:,i),'LineWidth',1.0)
        plot(T50/60,Y50(:,i),'LineWidth',1.0)

        xlabel('Time [min]')
        ylabel(sprintf('y_%d [cm]',i))

        legend('10%','25%','50%','Location','best')

        grid on

    end

    sgtitle(sprintf('F_2 step responses - %s',noiseNames{n}));


    % Normalize measured responses
    S10 = (Y10 - y0) / dF2_10;
    S25 = (Y25 - y0) / dF2_25;
    S50 = (Y50 - y0) / dF2_50;

    % Plot normalized responses
    figure("Color","w");

    for i = 1:2

        subplot(2,1,i)

        plot(T10/60, S10(:,i), 'LineWidth', 1.0)
        hold on

        plot(T25/60, S25(:,i), 'LineWidth', 1.0)
        plot(T50/60, S50(:,i), 'LineWidth', 1.0)

        xlabel('Time [min]')
        ylabel(sprintf('Normalized response S_{%d2}',i))

        legend('10% step','25% step','50% step', ...
               'Location','best')

        grid on

    end

    sgtitle(sprintf('Normalized F_2 responses - %s', ...
                    noiseNames{n}));

end

%% Identification from normalized steps

% close all;

Tsid = 5;                              % [s] uniform sampling
tu   = (0:Tsid:tend)';
Nu   = numel(tu);

S1 = interp1(T10_F1, S10_F1(:,1:2), tu);      % N x 2  (response to F1)
S2 = interp1(T10_F2, S10_F2(:,1:2), tu);      % N x 2  (response to F2)
Sid = {S1, S2};

np = [1 2;      % G11 G12
      2 1];      % G21 G22

opt = tfestOptions('InitialCondition','zero');
G = tf(zeros(2,2));
fitPct = nan(2,2);

for j = 1:2
    for i = 1:2
        nPre = 10;  % >2 (maximum order)
        yref = Sid{j}(:,i); 
        data = iddata( ...
            [zeros(nPre,1); yref], ...
            [zeros(nPre,1); ones(Nu,1)], ...
            Tsid);
        data.InterSample = 'zoh';
        Gij   = tfest(data, np(i,j), 0, 'Ts', 0, opt);
        G(i,j) = tf(Gij);
        fitPct(i,j) = 100*(1-goodnessOfFit(lsim(Gij,ones(Nu,1),tu), Sid{j}(:,i),'NRMSE'));
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
    for i = 1:2
        subplot(2,1,i); hold on; grid on

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

Tm = 5;                         % Sampling time [s]
Nm = 60;                        % Number of matrices H_1,...,H_Nm
Gdisc = c2d(G, Tm, 'zoh');         % Discretization with piecewise-constant inputs
tm = (0:Nm)' * Tm;

hImpulse = impulse(Gdisc, tm);
Hmodel = Tm * hImpulse(2:end,:,:); % Remove the direct term at t = 0 and correct the scaling

% Markov coefficients from normalized nonlinear step responses
S1m = interp1(T10_F1, S10_F1(:,1:2), tm);
S2m = interp1(T10_F2, S10_F2(:,1:2), tm);

Hdata = zeros(Nm,2,2);

Hdata(:,:,1) = diff(S1m,1,1);
Hdata(:,:,2) = diff(S2m,1,1);

% Plot each input-output channel
ell = (1:Nm)';

figure('Color','w');

for i = 1:2
    for j = 1:2
        subplot(2,2,(i-1)*2+j);

        stem(ell, Hmodel(:,i,j), ...
             'filled', 'MarkerSize',3);

        grid on
        ylabel(sprintf('H_{%d%d}',i,j));
    end
end

sgtitle(sprintf('Markov parameters, T_m = %g s', Tm));



%% Keep results

Gidentified = G;
save('identifiedModel.mat', 'Gidentified', 'us', 'p');

Hidentified = Hmodel;
Hstep = Hdata;
save('markovProblem4.mat', 'Hidentified', 'Hstep', 'Tm', 'Nm', 'p', 'us');

sysIdentifiedDisc = c2d(Gidentified, Ts, 'zoh');
[Aid, Bid, Cyid, Did] = ssdata(sysIdentifiedDisc);