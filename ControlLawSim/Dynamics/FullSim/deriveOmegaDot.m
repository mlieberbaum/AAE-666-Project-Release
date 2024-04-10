clear
close all
clc


% This script derives the three scalar differential equations for body rate
% derivatives:
%
% J*wdot = -w X Jw + u + tau_gg
%
% where J is the inertia matrix, w is the body rate vector, u is the torque
% input, and tau_gg is the gravity gradient torque, approximated by the 
% first term in the power series.  Here, the torque input is assumed 
% constant throughout the propagation step, simulating a discrete 
% controller

syms J11 J22 J33 J12 J13 J23 J11e J22e J33e J12e J13e J23e w1 w2 w3 x1 x2 x3 u1 u2 u3 muEarth real;


J = [J11 J12 J13
     J12 J22 J23
     J13 J23 J33];

w = [w1
     w2
     w3];

u = [u1
     u2
     u3];

r = [x1
     x2
     x3];

% Gravity-Gradient Torque (r^5 in the denominator instead of r^3 so we can
% use the full position vectors, not the unit position vectors, in the
% cross product)
taugg = ( 3.0 * muEarth / ( (x1*x1 + x2*x2 + x3*x3)^(5/2) ) ) * ...
    cross(r, J*r);

wdot = inv(J) * (-cross(w, J*w) + u + taugg)

