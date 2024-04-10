clear
close all
clc

addpath(genpath('.\Dynamics'))
addpath(genpath('.\Functions'))
addpath(genpath('.\InputFiles'))



%% LOAD INPUT FILE

inputData = BongWieDiscreteFullSim();




%% RUN SIMULATION

data = ControlLawSim(inputData);



%% ORBIT ANALYSIS PLOTS
figure;
set(gcf, 'Color', 'w')
set(gcf, 'Position', [870 530 620 690])
tl = tiledlayout(3, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
Torbit = 7741.8395;

nexttile;
hold on
grid on
plot(data.t ./ Torbit, (inputData.ctrl.qc(1:3).' - data.q(:,1:3)) * 180/pi * 3600);
xlabel('Orbit Number')
ylabel('arc seconds')
title('Pointing Error vs. Orbit Number')
xlim([0 5])
legend('x', 'y', 'z', 'Location', 'Best')

nexttile;
hold on
grid on
plot(data.t ./ Torbit, 0 - data.w);
xlabel('Orbit Number')
ylabel('rad/s')
title('Body Rate Error vs. Orbit Number')
xlim([0 5])
ylim([-2e-7 2e-7]);
legend('x', 'y', 'z', 'Location', 'Best')

nexttile;
hold on
grid on
plot(data.t ./ Torbit, data.u);
xlabel('Orbit Number')
ylabel('N m')
title('Control Input vs. Orbit Number')
xlim([0 5])
legend('x', 'y', 'z', 'Location', 'Best')

saveas(gcf, 'FullOrbitSim.svg', 'svg')