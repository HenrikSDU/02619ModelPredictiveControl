%% MPC Week 2
clc; clear; close all;
%% Parameters
p = FourTankStandardParams();

%% Exercise 1 
clc
% Linearization and steady-state

% .1 Compute steady-state

F1 = 250; % [cm^3/s]
F2 = 325; % [cm^3/s]


xs0 = 5000*ones(4,1); % Initial guess of steady state

us = [F1 F2]; % Input for steady-state

% Solve for steady-state
disp("Steady-State Values")
xs = fsolve(@FourTankSystemWrap,xs0,[],us,p)
zs = FourTankSystemOutput(xs,p)
ys = FourTankSystemSensor(xs,p)





