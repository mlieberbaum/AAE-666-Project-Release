function inputData = BongWieDiscreteFullSim()
    
    
    % Simulation parameters
    inputData.sim.tf = 38710;
    inputData.sim.dt = 0.1;


    % Initial state
    inputData.x0.r = [6678137.0, 0.0, 0.0].';
    inputData.x0.v = [0.0, 7361.21593216773, 4250].';
    inputData.x0.w = [0.0, 0.0, 0.0].';
    inputData.x0.q = [0.4826, 0.1075, 0.6831, 0.5375].';
    inputData.x0.q = inputData.x0.q ./ norm(inputData.x0.q);


    % Controller parameters (Section IV Case 4)
    inputData.ctrl.K = diag([110,110,110]);
    inputData.ctrl.D = 0.316 .* diag([1200, 2200, 3100]);
    
    inputData.ctrl.Jest = diag([1200, 2200, 3100]);
    inputData.ctrl.mu = 0.9;
    inputData.ctrl.type = 'BWDISCFULL';

    inputData.ctrl.qc = [0.4826; 0.1075; 0.6831; 0.5375];
    inputData.ctrl.qc = inputData.ctrl.qc ./ norm(inputData.ctrl.qc);

    inputData.ctrl.nZOH = 2;  % Number of time steps to do the ZOH.  
                               % Must be >= 1 (do NOT set to 0)


    % Spacecraft plant model parameters
    inputData.spacecraft.J = [1200   100  -200
                               100  2200   300
                              -200   300  3100] * 0.9;
    
end