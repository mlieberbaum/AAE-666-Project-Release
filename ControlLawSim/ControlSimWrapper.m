clear
close all
clc

addpath('.\Dynamics')
addpath('.\Functions')
addpath('.\InputFiles')



%% LOAD INPUT FILE

inputData = BongWieCtrlLawDemo();




%% RUN SIMULATION

data = ControlLawSim(inputData);




%% MAKE PLOTS

figure;
plot(data.t, data.q);

figure
plot(data.t, data.w);

figure
plot(data.t,data.u);