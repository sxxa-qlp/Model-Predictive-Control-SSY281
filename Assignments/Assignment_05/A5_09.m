clc;clear;close all;
%% Q1
% you are supposed to find Matrix S such that V(x(k)) is a Lyapunov
% function that V is decreasing (except at the origin) in the report;
%In addition, define matrix S here as well.

A = [0.5 1; -0.1 0.2];
Q = eye(2);

S= dlyap(A,Q)
% fp.m2latex(S);

%% Q2

A = [1 1; -1 5];
B = [1; 0];
Q = eye(2);
R = 1;

[P,lambda,K] = dare(A,B,Q,R)



%% Q3 
% part a:
% Define N (the shortest horizon that ...) here as N3. You can use DP_XX.m
% that you have writen in previous assignments. Do note that when I run
% your code, I should be able to see the result and just writing a number
% would not be enough. mention this N in your report as well.
%N3=...
% part b:
% explain in the report
% part c:
% Fill in the function Pf_XX.m in which XX is your group number. Motivate
% the concept behind your code in the report.
% part d:
% you can use trial and error to find this R; just provide the R that works
% and check the stability by checking the eigenvalues of the closed loop
% system with this new R; define it in the code as Rnew
%Rnew=...
%% Q4 
% write your proof in the report
%% Q5
% answer to the question in the report. Do note that you can verify your
% answer by checking it numerically in Matlab (this is just for your own
% and you should not provide any code regarding this)
%% Q6
% answer in the report






