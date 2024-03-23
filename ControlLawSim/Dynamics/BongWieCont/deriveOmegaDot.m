clear
close all
clc


% This script derives the three scalar differential equations for body rate
% derivatives:
%
% J*wdot = -w X Jw + u
%
% where J is the inertia matrix, w is the body rate vector, and u is the
% torque input.  Here, the torque input follows the control law:
%
% u = w X Jw - D*w - K*qe
%
% where D and K are matrices and qe is the eigenaxis component of the
% quaternion

syms J11 J22 J33 J12 J13 J23 J11e J22e J33e J12e J13e J23e mu D1 D2 D3 K1 K2 K3 w1 w2 w3 q1 q2 q3 real;


J = [J11 J12 J13
     J12 J22 J23
     J13 J23 J33];

Je = [J11e J12e J13e
      J12e J22e J23e
      J13e J23e J33e];

D = [D1   0   0
      0  D2   0
      0   0  D3];

K = [K1   0   0
      0  K2   0
      0   0  K3];

w = [w1
     w2
     w3];

qe = [q1
      q2
      q3];

u = mu*cross(w, Je*w) - D*w - K*qe;

wdot = inv(J) * (-cross(w, J*w) + u)

