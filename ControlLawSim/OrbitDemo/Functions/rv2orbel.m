function orbEl = rv2orbel(r, v)

    % Function to compute orbital elements from r and v
    
    r2d = 180 / pi;
    
    mu = 398600.4415;
    rnorm = norm(r);
    vnorm = norm(v);
    
    H = cross(r, v);
    
    S1 = (1 / mu) * (vnorm^2 - (mu / rnorm));
    S2 = (1 / mu) * dot(r, v);
    
    E = S1 .* r - S2 .* v;
    
    N = [-H(2), H(1), 0];
    
    p = (norm(H) .^ 2) ./ mu;
    
    orbEl.e = norm(E);
    orbEl.a = p / (1 - (orbEl.e)^2);
    orbEl.i = acos(H(3) / norm(H)) * r2d;
    orbEl.T = 2 * pi * sqrt((orbEl.a) ^3 / mu);
    
    if orbEl.i == 0
        orbEl.LAN = 0;
        orbEl.omega = 0;
        orbEl.theta = 0;
    else
        if N(2) > 0
            orbEl.LAN = acos(N(1) / norm(N)) * r2d;
        else
			orbEl.LAN = 360.0 - (acos(N(1) / norm(N)) * r2d);
        end

		if E(3) > 0
			orbEl.omega = acos((dot(N, E)) / (norm(N) * norm(E))) * r2d;
		else 
			orbEl.omega = 360.0 - (acos((dot(N, E)) / (norm(N) * norm(E))) * r2d);
        end

		if (dot(r, v) > 0.0) 
			orbEl.theta = acos((dot(E, r)) / (norm(E) * norm(r))) * r2d;
		else 
			orbEl.theta = 360.0 - (acos((dot(E, r)) / (norm(E) * norm(r))) * r2d);
        end
        
    end
    
    
    
    
    
end
    