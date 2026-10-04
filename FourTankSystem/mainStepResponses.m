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

tf = 20*60;     % [s]

stepSizes = [0.10 0.25 0.50];

%% Simulate steps in F1

[T10_F1, X10_F1, H10_F1] = simulateStep(us, xs, p, 0.10, 1, tf, d);

[T25_F1, X25_F1, H25_F1] = simulateStep(us, xs, p, 0.25, 1, tf, d);

[T50_F1, X50_F1, H50_F1] = simulateStep(us, xs, p, 0.50, 1, tf, d);

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

%% Simulate steps in F2

[T10_F2, X10_F2, H10_F2] = simulateStep(us, xs, p, 0.10, 2, tf, d);

[T25_F2, X25_F2, H25_F2] = simulateStep(us, xs, p, 0.25, 2, tf, d);

[T50_F2, X50_F2, H50_F2] = simulateStep(us, xs, p, 0.50, 2, tf, d);

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