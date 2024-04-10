clear
close all
clc

addpath(genpath('.\Dynamics'))
addpath(genpath('.\Functions'))
addpath(genpath('.\InputFiles'))



%% LOAD INPUT FILE

inputData = BongWieDiscrete();




%% RUN SIMULATION

data = ControlLawSim(inputData);




%% MAKE PLOTS

figure;
plot(data.t, data.q);

figure
plot(data.t, data.w);

figure
hold on
plot(data.t, data.u, '--');
set(gca,'colororderindex',1);
plot(data.t, data.ua)

