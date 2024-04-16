%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This script reproduces the plots in Fig. 1 and Fig. 2 of the journal
% article:
% 
% Wie, B., Weiss, H., and Arapostathis, A., “Quaternion Feedback Regulator 
% for Spacecraft Eigenaxis Rotations,” Journal of Guidance, Control, and 
% Dynamics, Vol. 12, No. 3, 1989, pp. 375-380.doi: 10.2514/3.20418
% 
% The simulation input conditions, per the journal article, are:
% 
% Spacecraft Inertia Matrix, with 10% error (kg m^2):
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
close all
clc


addpath(genpath('.\Dynamics'))
addpath(genpath('.\Functions'))
addpath(genpath('.\InputFiles'))



%% RUN THE SIMULATIONS

inputData = BongWieCtrlLawDemo();

% Case 1: K = diag([201 110 78])
inputData.ctrl.K = diag([201 110 78]);
data{1} = ControlLawSim(inputData);

% Case 2: K = diag([110 110 110])
inputData.ctrl.K = diag([110 110 110]);
data{2} = ControlLawSim(inputData);

% Case 3: K = diag([72 110 204])
inputData.ctrl.K = diag([72 110 204]);
data{3} = ControlLawSim(inputData);

% Case 4: K = diag([60 110 155])
inputData.ctrl.K = diag([60 110 155]);
data{4} = ControlLawSim(inputData);



%% QUATERNION PLOT

figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [640 700 730 530]);

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

lineStyles = {'-.', ':', '--', '-'};

ylimits = [-0.2 0.6
           -0.4 0.6
           -0.2 0.8
            0.0 1.0];

for idx = 1:4

    nexttile;
    hold on
    grid on

    for idx2 = 1:4
        plot(data{idx2}.t, data{idx2}.q(:,idx), 'k', 'LineStyle', lineStyles{idx2});
    end

    xlabel('Time (sec)')
    ylabel(['q', num2str(idx)])
    xlim([0 100])
    xticks(0:50:100);
    ylim(ylimits(idx,:))

    legend('Case 1', 'Case 2', 'Case 3', 'Case 4', 'Location', 'Best')

end




%% BODY RATE PLOT

figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [640 700 730 530]);

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

lineStyles = {'-.', ':', '--', '-'};

ylimits = [-0.3  0.1
           -0.1  0.05
           -0.15 0.05];

yt = {-0.3  : 0.1  : 0.1
      -0.1  : 0.05 : 0.05
      -0.15 : 0.05 : 0.05};

for idx = 1:3

    nexttile;
    hold on
    grid on

    for idx2 = 1:4
        plot(data{idx2}.t, data{idx2}.w(:,idx), 'k', 'LineStyle', lineStyles{idx2});
    end

    xlabel('Time (sec)')
    ylabel(['\omega', num2str(idx), ' (rad/s)'])
    xlim([0 100])
    xticks(0:50:100);
    ylim(ylimits(idx,:));
    yticks(yt{idx});

    legend('Case 1', 'Case 2', 'Case 3', 'Case 4', 'Location', 'Best')

end





%% CONTROL INPUT PLOT

figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [640 700 730 530]);

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

lineStyles = {'-.', ':', '--', '-'};

ylimits = [-150  50
           -100  50
           -150  50];

for idx = 1:3

    nexttile;
    hold on
    grid on

    for idx2 = 1:4
        plot(data{idx2}.t, data{idx2}.u(:,idx), 'k', 'LineStyle', lineStyles{idx2});
    end

    xlabel('Time (sec)')
    ylabel(['u', num2str(idx)])
    xlim([0 100])
    xticks(0:50:100);
    ylim(ylimits(idx,:));

    legend('Case 1', 'Case 2', 'Case 3', 'Case 4', 'Location', 'Best')

end

