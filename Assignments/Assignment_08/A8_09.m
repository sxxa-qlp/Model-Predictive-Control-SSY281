clear;close all;clc;
%  Write a code that does the simulation and plots the system output and
%  input with respect to time.
% Your plots should have meaningful labels and simulates the system after
% the convergence to the final set as well to shows the system remains in
% it. Explain the concept that you have used to design the controller in
% your report, provide the simulation outcomes, and explain your
% observations.


%% Model parameters

LS = 1;
dS = 0.02;
JM = 0.5;
betaM = 0.1;
R = 20;
kT = 10;
rho = 20;
kth = 1280.2;
JL = 50*JM;
betaL = 25;

% x = [thetaL dthetaL thetaM dthetaM]
% y = torsional torque T
% u = input DC voltage V
A = [0 1 0 0;
     -kth/JL -betaL/JL kth/(rho*JL) 0;
     0 0 0 1;
     kth/(rho*JM) 0 -kth/(rho^2*JM) -(betaM+kT^2/R)/JM];
B = [0 0 0 kT/(R*JM)].';
C = [kth 0 -kth/rho 0];


x0 = [0 2.5 0 75]';

% input and output constraints (lower and upper bound)
T_b = [-157 157];
V_b = [-200 200];


%% Create MPC model

model = LTISystem('A', A, 'B', B, 'C', C);
model.y.min = T_b(1);
model.y.max = T_b(2);
model.u.min = V_b(1);
model.u.max = V_b(2);

Q=eye(4); 
R=100;

model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);


X = Polyhedron('A',[C;-C],'B',[T_b(2); -T_b(1)]);
U = Polyhedron('lb',model.u.min,'ub',model.u.max);











