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

stable = false;
N2 = 0;
while ~stable
    N2 = N2+1;
    [K0,P0] = DP_09(A,B,N2,Q,R,Pf);
    if all(abs(eig(A-B*K0))<=1)
        stable = true;
    end
end

Pf
N2
eig(A-B*K0)
P0
K0

%N2= the shortest N!

%% Q3: Define Pf3 as the solution to the Riccati Equation; define N3 that gives the shortest N, i.e. N3=min(N) subject to the system stability

[Pf3,~,~] = dare(A,B,Q,R);

stable = false;
N3 = 0;
while ~stable
    N3 = N3+1;
    [K0,P0] = DP_09(A,B,N3,Q,R,Pf3);
    if all(abs(eig(A-B*K0))<=1)
        stable = true;
    end
end

Pf3
N3
eig(A-B*K0)
P0
K0

%N3= the shortest N!
%Pf3= the shortest N!

%% Q4: Fill the BS_XX.m function using the batch solution approach.

%% Q5: Find the shortest N that stabilizes the system using BS_XX.m; define N5 that gives the shortest N, i.e. N5=min(N) subject to the system stability

stable = false;
N5 = 0;
while ~stable
    N5 = N5+1;
    [K0,P0] = BS_09(A,B,N5,Q,R,Pf);
    if all(abs(eig(A-B*K0))<=1)
        stable = true;
    end
end

Pf
N5
eig(A-B*K0)
P0
K0

%N5=the shortest N!

%% Q6: Use BS_XX.m or DP_XX.m and simulate the system for 20 steps; plot the inputs and the states for these four cases.

tf=20;
x0=[1;0];

% R=0.5;N=5;
% R=0.5;N=15;
% R=0.05;N=5;
% R=0.05;N=15;

R = [0.5, 0.5, 0.05, 0.05];
N = [5, 15, 5, 15];

f1 = figure('Color','white','Position',[149.0000  237.8000  836.8000  358.4000]); hold on, grid on;
xlabel 'time-step k', ylabel 'x(k)'
f2 = figure('Color','white','Position',[149.0000  237.8000  673.6000  358.4000]); hold on, grid on;
xlabel 'time-step k', ylabel 'u(k)'
clr = lines(20);

for i=1:numel(R)
    K0 = DP_09(A,B,N(i),Q,R(i),Pf);
    [x, u] = sim_lin(A, B, K0, x0, tf);
    figure(f1)
    stairs(0:tf,x(1,:)','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('x(1) R=%.2f N=%d',R(i),N(i)));
    stairs(0:tf,x(2,:)','-.','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('x(2)'));
    figure(f2)
    stairs(0:(tf-1),u','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('u R=%.2f N=%d',R(i),N(i)));
end
figure(f1)
legend('Location','southeastoutside');
% fp.savefig('q6_x');
figure(f2)
legend('Location','southeast');
% fp.savefig('q6_u');


%% Q7: Fill the CRHC1_XX.m function
%% Q8: Fill the CRHC2_XX.m function
%% Q9: Solve Q6 using CRHC1_XX.m or CRHC2_XX.m considering the given constraints for 100 sample times

tf=100;
x0=[1;0];
R = [0.5, 0.5, 0.05, 0.05];
N = [5, 15, 5, 15];

% F1*x + G1*u =  h1
F1_i = zeros(2);
G1_i = [0;0];
h1_i = zeros(2,1);

% F2*x + G2*u <= h2
F2_i = [0 1;
       0 -1;
       0 0;
       0 0];
G2_i = [0;
        0;
        1;
       -1];
h2_i = [0.5;
        0.5;
        0.7;
        0.7];


f1 = figure('Color','white','Position',[149.0000  237.8000  836.8000  358.4000]); hold on, grid on;
xlabel 'time-step k', ylabel 'x(k)'
f2 = figure('Color','white','Position',[149.0000  237.8000  673.6000  358.4000]); hold on, grid on;
xlabel 'time-step k', ylabel 'u(k)'
clr = lines(20);

for i=1:numel(R)
    F2 = kron(eye(N(i)),F2_i);
    G2 = kron(eye(N(i)),G2_i);
    h2 = kron(ones(N(i),1),h2_i);
    F1 = kron(eye(N(i)),F1_i);
    G1 = kron(eye(N(i)),G1_i);
    h1 = kron(ones(N(i),1),h1_i);

    [x,u] = sim_MPC(A,B,N(i),Q,R(i),Pf,F1,G1,h1,F2,G2,h2,x0,tf);
    figure(f1)
    stairs(0:tf,x(1,:)','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('x(1) R=%.2f N=%d',R(i),N(i)));
    stairs(0:tf,x(2,:)','-.','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('x(2)'));
    figure(f2)
    stairs(0:(tf-1),u','Color',clr(i,:),'LineWidth',2,'DisplayName',sprintf('u R=%.2f N=%d',R(i),N(i)));
end
figure(f1)
legend('Location','southeastoutside');
% fp.savefig('q9_x');
figure(f2)
legend('Location','southeast');
% fp.savefig('q9_u');


%% Help functions

function [x,u] = sim_MPC(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0,tf)
    x = x0;
    u = [];
    for t=1:tf
        [Z,~]=CRHC1_09(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x(:,t));
        u(:,t) = Z(end-N+1,:);        
        x(:,t+1) = A*x(:,t) + B*u(:,t);
    end
end

function [x,u] = sim_lin(A, B, K0, x0, tf)
    x = x0;
    u = [];
    for t=1:tf
        u(:,t) = -K0*x(:,t);
        x(:,t+1) = A*x(:,t) + B*u(:,t);
    end
end



