clear;clc;
%% Q1: Fill the DP_XX.m function using the dynamic programic approach.

%% Q2: Find the shortest N that stabilizes the system using DP_XX.m ; define N2 that gives the shortest N, i.e. N2=min(N) subject to the system stability
A=[1.0025 0.1001;0.05 1.0025];
B=[0.005;0.1001];
C=[1 0];
D=0;
Q=[5 0;0 1];
Pf=[5 0;0 1];
R=0.5;

%N2= the shortest N!

%% Q3: Define Pf3 as the solution to the Riccati Equation; define N3 that gives the shortest N, i.e. N3=min(N) subject to the system stability


%N3= the shortest N!
%Pf3= the shortest N!

%% Q4: Fill the BS_XX.m function using the batch solution approach.


%% Q5: Find the shortest N that stabilizes the system using BS_XX.m; define N5 that gives the shortest N, i.e. N5=min(N) subject to the system stability


%N5=the shortest N!

%% Q6: Use BS_XX.m or DP_XX.m and simulate the system for 20 steps; plot the inputs and the states for these four cases.
tf=20;
x0=[1;0];

% R=0.5;N=5;
% R=0.5;N=15;
% R=0.05;N=5;
% R=0.05;N=15;

%% Q7: Fill the CRHC1_XX.m function
%% Q8: Fill the CRHC2_XX.m function
%% Q9: Solve Q6 using CRHC1_XX.m or CRHC2_XX.m considering the given constraints for 100 sample times
tf=100;
x0=[1;0];
% R=0.5;N=5;
% R=0.5;N=15;
% R=0.05;N=5;
% R=0.05;N=15;



