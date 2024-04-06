function inputData = BongWieTimeDelay()
    
    
    % Simulation parameters
    inputData.sim.tf = 1000;
    inputData.sim.dt = 0.01;


    % Initial state
    inputData.x0.r = [6678137.0, 0.0, 0.0].';
    inputData.x0.v = [0.0, 7361.21593216773, 4250].';
    inputData.x0.w = [0.0, 0.0, 0.0].';
    inputData.x0.q = [0.0, 0.0, 0.0, 1.0].';
    inputData.x0.q = inputData.x0.q ./ norm(inputData.x0.q);


    % Controller parameters (Section IV Case 4)
    inputData.ctrl.K = diag([60,110,155]);
    inputData.ctrl.D = 0.316 .* diag([1200, 2200, 3100]);
    
    inputData.ctrl.Jest = diag([1200, 2200, 3100]);
    inputData.ctrl.mu = 0.9;
    inputData.ctrl.type = 'BWDISC';
    inputData.ctrl.delay = 407;  % Sim frames of delay (divide by dt to get delay in seconds)
    inputData.ctrl.nZOH = 1;

    % 0.1 rad +X step rotation
    inputData.ctrl.qc = [sin(1/2) ; 0 ; 0 ; cos(1/2)];

    % Normalize quaternion command
    inputData.ctrl.qc = inputData.ctrl.qc ./ norm(inputData.ctrl.qc);


    % Spacecraft plant model parameters
    inputData.spacecraft.J = [1200     0     0
                                 0  2200     0
                                 0     0  3100];
    
end