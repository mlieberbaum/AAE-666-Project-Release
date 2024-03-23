clear
close all
clc


% This script derives the three scalar differential equations for body rate
% derivatives:
%
% J*wdot = -w X Jw + u
%
% where J is the inertia matrix, w is the body rate vector, and u is the
% torque input.  Here, the torque input is assumed constant throughout the
% propagation step, simulating a discrete controller

syms J11 J22 J33 J12 J13 J23 w1 w2 w3 u1 u2 u3 real;


J = [J11 J12 J13
     J12 J22 J23
     J13 J23 J33];

w = [w1
     w2
     w3];

u = [u1
     u2
     u3];

wdot = inv(J) * (-cross(w, J*w) + u)