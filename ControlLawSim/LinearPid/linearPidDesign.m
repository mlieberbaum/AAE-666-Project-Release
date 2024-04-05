clear
close all
clc


% Script to design a linear PD and PID controller about the x-axis.  The y
% and z axes can be designed in a similar manner.
%
% References: 
% [1] Wie, B., Weiss, H., and Arapostathis, A., "Quaternion Feedback 
%     Regulatorfor Spacecraft Eigenaxis Rotations," Journal of Guidance, 
%     Control, and Dynamics, Vol. 12, No. 3, 1989, pp. 375-380.
%     doi: 10.2514/3.20418


% Spacecraft Inertia
J = 1200;


% Quaternion and Rate gain feedback gains: Case 4 from [1]
K = 0.05 * J;
D = 0.316 * J;


% Compute proportional gain for linear PD control, using the small angle
% approximation (2qi ~= thi)
KpPD = K / 2;


% Rate gain is identical
KdPD = D(1,1);


% Compute PD controller open loop and closed loop transfer function
s = tf('s');
P = 1/J/s^2;
Cpd = (KpPD + KdPD*s);
Gol = Cpd*P;
Gcl = feedback(Gol,1);
marginPd = allmargin(Gol);


% Plot PD controller parameters
fh = figure;
hold on;
grid on;
step(Gcl)

figure;
margin(Gol)
legend('PD Controller')


% Display PD Controller Parameters
fprintf('----------------------------------------\n\n');
fprintf(' PD Controller Parameters:\n')
fprintf(['   Kp = ', num2str(KpPD, '%6.4f'), '\n'])
fprintf(['   Kd = ', num2str(KdPD, '%6.4f'), '\n\n'])
fprintf(['   Delay Margin = ', num2str(marginPd.DelayMargin, '%6.2f'), ' sec\n\n'])


% Get closed loop poles from PD controller
p = pole(Gcl);
p = [p; real(p(1))];
pcoeff = poly(p);


% Compute Ki to place third closed loop pole on real axis equidistant from
% imaginary axis of the two PD controller closed loop poles (pole placement
% for double integrator plant with PID controller)
KdPID = pcoeff(2) * J;
KpPID = pcoeff(3) * J;
KiPID = pcoeff(4) * J;

Cpid = (KpPID + KdPID*s + KiPID/s);
Gol = Cpid*P;
Gcl = feedback(Gol,1);
marginPid = allmargin(Gol);

figure;
margin(Gol);
legend('PID Controller')

figure(fh);
step(Gcl);
legend('PD', 'PID', 'Location', 'Best')


% Display PID Controller Parameters
fprintf('----------------------------------------\n\n');
fprintf(' PID Controller Parameters:\n')
fprintf(['   Kp = ', num2str(KpPID, '%6.4f'), '\n'])
fprintf(['   Ki = ', num2str(KiPID, '%6.4f'), '\n'])
fprintf(['   Kd = ', num2str(KdPID, '%6.4f'), '\n\n'])
fprintf(['   Delay Margin = ', num2str(marginPid.DelayMargin, '%6.2f'), ' sec\n\n'])




