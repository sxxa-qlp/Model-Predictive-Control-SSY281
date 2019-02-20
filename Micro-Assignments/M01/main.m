clear all; close all; clc;

x = sym('x',[2,1]);
syms u
ys = 1;

A = [0.5 0.2; -0.2 0.5];
B = [0; 1];
C = [1 0];

eq = [x == A*x + B*u;
      ys == C*x];

answ = solve(eq);

fprintf('Steady states x_{ss}=[%.2f %.2f], u_{ss}=[%.2f]\n', answ.x1, answ.x2, answ.u)


