%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This script compares the nonlinear control law from the reference below
% to two linear control laws.
% 
% Wie, B., Weiss, H., and Arapostathis, A., “Quaternion Feedback Regulator 
% for Spacecraft Eigenaxis Rotations,” Journal of Guidance, Control, and 
% Dynamics, Vol. 12, No. 3, 1989, pp. 375-380.doi: 10.2514/3.20418
% 
% The simulation input conditions are:
% 
% Spacecraft Inertia Matrix (kg m^2):
% 
%   J = [1200   100  -200
%         100  2200   300
%        -200   300  3100] * 0.9
% 
% Initial quaternion (note: this was normalized to ensure magnitude 1):
%
%   q0 = [0.57, 0.57, 0.57, 0.159]
%
% Commanded quaternion:
%
%   qc = [0, 0, 0, 1]
%
% Initial body rates (rad/s):
%
%   w0 = [0.01, 0.01, 0.01]
%
% Estimated inertia (10% error, zero diagonal elements assumed):
%
%   Jest = [1200     0     0
%              0  2200     0
%              0     0  3100]
%
% Control gain matrix D:
%
%   D = [379.2       0       0
%            0   695.2       0
%            0       0   979.6]
%
% Gyroscopic torque offset scale factor:
%
%   mu = 0.9
% 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
close all
clc


addpath(genpath('.\Dynamics'))
addpath(genpath('.\Functions'))
addpath(genpath('.\InputFiles'))



%% RUN THE SIMULATIONS

% Case 1: Continuous
inputData = BongWieCtrlLawDemo();
data{1} = ControlLawSim(inputData);

% Initialize digital controller simulation input
inputData = BongWieDiscrete();

% Case 2: 1 second ZOH
inputData.ctrl.nZOH = 100;
data{2} = ControlLawSim(inputData);

% Case 3: 2 second ZOH
inputData.ctrl.nZOH = 200;
data{3} = ControlLawSim(inputData);

% Case 4: 3 second ZOH
inputData.ctrl.nZOH = 300;
data{4} = ControlLawSim(inputData);

% Case 5: 4 second ZOH
inputData.ctrl.nZOH = 400;
data{5} = ControlLawSim(inputData);



figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [640 700 730 530]);

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for idx = 1:4

    nexttile;
    hold on
    grid on

    for idx2 = 1:numel(data)
        plot(data{idx2}.t(1:10:end), data{idx2}.q(1:10:end,idx));
    end

    xlabel('Time (sec)')
    ylabel(['q', num2str(idx)])
    
    if idx == 4
        legend('Continuous', '1 Hz ZOH', '0.5 Hz ZOH', '0.33 Hz ZOH', '0.25 Hz ZOH', 'Location', 'Best')
    end

end

saveas(gcf, 'DigitalCtrlSimQ.svg', 'svg')


%% BODY RATE PLOT

figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [640 700 730 530]);

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for idx = 1:3

    nexttile;
    hold on
    grid on

    for idx2 = 1:numel(data)
        plot(data{idx2}.t(1:10:end), data{idx2}.w(1:10:end,idx));
    end

    xlabel('Time (sec)')
    ylabel(['\omega', num2str(idx), ' (rad/s)'])

    if idx == 3
        lh = legend('Continuous', '1 Hz ZOH', '0.5 Hz ZOH', '0.33 Hz ZOH', '0.25 Hz ZOH', 'Location', 'Best');
        set(lh, 'Position', [0.6409 0.1913 0.1621 0.1497])
    end

end

saveas(gcf, 'DigitalCtrlSimOmega.svg', 'svg')
