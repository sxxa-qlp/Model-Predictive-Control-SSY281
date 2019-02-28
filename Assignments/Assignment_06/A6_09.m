clear;clc;close all;
%% Q1
% define H and V that are correspondent to the polyhedron for H and v representation. Plot the polyhedron and
% explain the difference in the report.

%H= define H
%V= define V


model = LTISystem('A', [1 1; 0 1], 'B', [1; 0.5]);
model.x.min = [-5; -5];
model.x.max = [5; 5];
model.u.min = -1;
model.u.max = 1;
Q = [1 0; 0 1];
model.x.penalty = QuadFunction(Q);
R = 1;
model.u.penalty = QuadFunction(R);
N = 5;
mpc = MPCController(model, N)

% x0 = [4; 0];
% u = mpc.evaluate(x0)
% [u, feasible, openloop] = mpc.evaluate(x0)



InvSet = model.invariantSet()
InvSet.plot()






%% Q2
% define Q and P, find the sum and the difference and plot the results.
% Sketch the plots in the report as well

%% Q3
% write a code that shows S is invariant and explain your approach in the
% report

%% Q4
% Fill in the Reach_XX function and Plot S and its one step reachable set.
% Note that you are not supposed to change the inputs and outputs of the
% function.

%% Q5
% Fill in the Pre_XX function and Plot S and its Pre set.
% Note that you are not supposed to change the inputs and outputs of the
% function.

%% Q6
% part 1: Fill in the function ShorterstN_XX.m and use it to find the shortest N
% that is feasible. Note that you are not supposed to change the inputs and outputs of the
% function.

% part 2: Fill in the function RHCXf_XX.m and use it to check the
% feasibility. Note that you are not supposed to change the inputs and outputs of the
% function.

% part 3: Plot the feasible sets for the initial condition in part 1 and 2
% and plot those sets. Answer to the rest of the question in the report. 
