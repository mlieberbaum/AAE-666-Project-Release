function inputData = OrbitDemo()
    
    
    % Simulation parameters
    inputData.sim.tf = 25000;
    inputData.sim.dt = 0.25;


    % Initial state
    inputData.x0.r = [6678137.0, 0.0, 0.0].';
    inputData.x0.v = [0.0, 7361.21593216773, 4250].';
    inputData.x0.w = [0.2, 1.0, 0.2].';
    inputData.x0.q = [0.0, 0.0, 0.0, 1.0].';


    % Controller parameters
    inputData.ctrl = [];
    inputData.ctrl.type = 'NONE';


    % Spacecraft plant model parameters
    inputData.spacecraft.J = diag([700 225 525]);
    
end