% AAE 532
% Mark Lieberbaum
% Problem Set 5
% Problem 2, Part a


function [thStar, eccentricAnomaly] = keplerProblemEllipse(tElapsed, a, mu, e)
    
    % Function to compute true anomaly given periapsis and time


    % First, compute mean motion and mean anomaly
    n = sqrt(mu/(a^3));
    M = n * tElapsed;


    % Next, solve iteratively for eccentric anomaly
    T = 2*pi*sqrt(a^3/mu);
    halfT = T/2.0;


    % Simple initial guesses
    if tElapsed < halfT
        En = pi/2;
    else
        En = -pi/2;
    end


    difference = 1e3;
    niter = 1;

    while abs(difference) > 1e-12
        
        Enplus1 = En - (En - e*sin(En) - M)/(1 - e*cos(En));
        En = Enplus1;
        Mn = En - e*sin(En);
        difference = abs(M - Mn);

        niter = niter + 1;

        if niter > 500
            error('Convergence error');
        end

    end

    % Convert to true anomaly
    c = sqrt((1+e)/(1-e));
    thStar = 2*atan(c*tan(En/2));
    eccentricAnomaly = En;

end