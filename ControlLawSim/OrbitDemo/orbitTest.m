clear
close all
clc


addpath(genpath('..'));
addpath('Functions')


%% SIMULATION

% Run the simulation
inputData = OrbitDemo();
data = ControlLawSim(inputData);

npts = numel(data.t);



%% ORBIT VERIFICATION

% Orbit initial position and velocity
r = [6678.137; 0; 0];
v = [0; 8.5*cosd(30); 8.5*sind(30)];

orbel = rv2orbel(r, v);

twoBodySolution = nan(npts, 6);

for idx = 1:npts

    t = data.t(idx);
    nOrbits = floor(t / orbel.T);

    % True anomaly
    th = keplerProblemEllipse(t - nOrbits*orbel.T, orbel.a, 398600.4415, orbel.e);

    % r and v
    [twoBodySolution(idx,1:3), twoBodySolution(idx,4:6)] = orbel2rv(orbel.a, orbel.e, orbel.i*pi/180, orbel.LAN*pi/180, orbel.omega*pi/180, th);

end

twoBodySolution = twoBodySolution .* 1000;



%% EPHEMERIS PLOTS

figure
hold on
grid on
set(gcf, 'Color', 'w')
set(gcf, 'Position', [190 390 1350 650]);

posErr = vecnorm((twoBodySolution(:,1:3) - data.r).');
plot(data.t ./ 86400, posErr)
xlabel('Days', 'FontSize', 12)
ylabel('meters', 'FontSize', 12)
title('Position Error to Two-Body Analytical Solution (meters)', 'FontSize', 14);


figure
hold on
grid on
set(gcf, 'Color', 'w')
set(gcf, 'Position', [190 390 1350 650]);

velErr = vecnorm((twoBodySolution(:,4:6) - data.v).');
plot(data.t ./ 86400, velErr)
xlabel('Days', 'FontSize', 12)
ylabel('m/s', 'FontSize', 12)
title('Velocity Error to Two-Body Analytical Solution (m/s)', 'FontSize', 14);


figure
hold on
grid on
set(gcf, 'Color', 'w')
set(gcf, 'Position', [190 390 1350 650]);
plot3(data.r(:,1), data.r(:,2), data.r(:,3))
hold on
[xSphere, ySphere, zSphere] = sphere(100);
surf(xSphere .* 6378137, ySphere .* 6378137, zSphere .* 6378137, 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeAlpha', 0.2)
axis equal
xlabel('ECI X')
ylabel('ECI Y')
zlabel('ECI Z')
view([24 22])
title('LEO Simulation')
subtitle('SMa = 8458.4 km, ecc = 0.21, inc = 30°')


%% ATTITUDE VERIFICATION

% From AAE 590 Spring 2021

% Radians to degrees
r2d = 180.0 / pi;


% Simulation start and end times
t0 = 0.0;           % [s]
tf = 102.0;          % [s]


% Moments of inertia
I1 = 700.0;          % [kg m^2]
I2 = 225.0;          % [kg m^2]
I3 = 525.0;          % [kg m^2]



% Differential equations with gravity torque (for Problem Set 10)
fdot = @(t,x) [ ((I2-I3)/I1) * x(2) * x(3);    % w1dot
                ((I3-I1)/I2) * x(3) * x(1);    % w2dot
                ((I1-I2)/I3) * x(1) * x(2);    % w3dot
                0.5 * ( x(1)*x(7) - x(2)*x(6) + x(3)*x(5));     % e1dot
                0.5 * ( x(1)*x(6) + x(2)*x(7) - x(3)*x(4));     % e2dot
                0.5 * (-x(1)*x(5) + x(2)*x(4) + x(3)*x(7));     % e3dot
               -0.5 * ( x(1)*x(4) + x(2)*x(5) + x(3)*x(6));     % e4dot
               ];
        

% Initial Conditions, for Problem Set 11.
x0 = [ 0.2;     % w1
       1.0;     % w2
       0.2;     % w3
       0.0;     % e1
       0.0;     % e2
       0.0;     % e3
       1.0      % e4
      ];


% Solve differential equations and initial conditions
S = odeset('RelTol', 1.0e-12, 'AbsTol', 1.0e-12);
[t, y] = ode45(fdot, [t0 tf], x0, S);



%% ATTITUDE PLOTS

figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [190 110 1350 930])

tl = tiledlayout(2,2);
title(tl, 'Quaternion Verification');

idxEnd = round(100 / inputData.sim.dt);

for idx = 1:4

    nexttile;
    hold on
    grid on
    plot(t,y(:,idx+3));
    plot(data.t(1 : idxEnd), data.q(1 : idxEnd, idx), '.');
    xlabel('Time (s)')
    title(['q_', num2str(idx)])

end



figure
set(gcf, 'Color', 'w')
set(gcf, 'Position', [190 110 1350 930])

tl = tiledlayout(2,2);
title(tl, 'Body Rate Verification');

for idx = 1:3

    nexttile;
    hold on
    grid on
    plot(t,y(:,idx));
    plot(data.t(1 : idxEnd), data.w(1 : idxEnd, idx), '.');
    xlabel('Time (s)')
    title(['\omega_', num2str(idx)])

end
