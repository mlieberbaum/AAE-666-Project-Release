clear
close all
clc


% This script derives the three scalar differential equations for body rate
% derivatives:
%
% J*wdot = -w X Jw
%
% where J is the inertia matrix, and w is the body rate vector.  No control
% torque input is assumed (i.e. a free tumble).

syms J11 J22 J33 J12 J13 J23 w1 w2 w3 u1 u2 u3 real;


J = [J11 J12 J13
     J12 J22 J23
     J13 J23 J33];

w = [w1
     w2
     w3];

wdot = inv(J) * (-cross(w, J*w))