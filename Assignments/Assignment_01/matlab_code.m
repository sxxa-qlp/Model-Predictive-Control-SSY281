clear;clc;close all;
%%You are supposed to fill this template. Remeber not to change variable
%%names!
%% Discrete Model
Ac=[0 1;0.5 0];Bc=[0;1];Cc=[1 0];Dc=0;

%define the sampling time here
wn = damp(Ac);
h = pi/wn(1);    % [s]

%Use given parameters and find A, B, and C in Question #1
sys=ss(Ac,Bc,Cc,Dc);
sysd=c2d(sys,h);
A = sysd.A;
B = sysd.B;
C = sysd.C;


%% Delayed model
% define Aa, Ba, Ca, Da according to Question #2

h = 0.1;
syms si
A = expm(Ac*h);
B = double(int(expm(Ac*si)*Bc,si,0,h));
C = Cc;

tau = 0.5*h;
syms tausym

B1 = double( expm(Ac*(h-tau)) * int(expm(Ac*tausym)*Bc,tausym,0,tau) );
B2 = double( int(expm(Ac*tausym)*Bc,tausym,0,h-tau) );

Aa = [A B1; 0*(Ac*Bc)' 0*Bc'*Bc];
Ba = [B2; 1];
Ca = [C 0];
Da = Dc;


%% Controllability and Observability
%find the rank of controllablity and observability matrices according to

%Question #3
% the following parameters should be the ranks of the matrices

sys2_c = rank(ctrb(Ac,Bc));
sys2_o = rank(obsv(Ac,Cc));
sys3_c = rank(ctrb(A,B));
sys3_o = rank(obsv(A,C));
sys4_c = rank(ctrb(Aa,Ba));
sys4_o = rank(obsv(Aa,Ca));

%Question 4 - find matrix C such that (2) is not observable
syms c1 c2
Csym = [c1 c2];
c2_nonobsv = solve(det([Csym; Csym*Ac]),c2);
c1_val = 1;
c2_val = double(subs(c2_nonobsv(1),c1,c1_val));
C_nonobsv = [c1_val c2_val];

svd(obsv(Ac,C_nonobsv))         % rank defficient
svd(obsv(A, C_nonobsv))         % rank defficient
svd(obsv(Aa, [C_nonobsv 0]))    % rank defficient



%% Controller design
lambda1=-4+6*1i;
lambda2=-4-6*1i;

%calculate desired poles for the discrete time system (3) and define them as p1 and
%p2 as it is asked in Question #6
p1 = exp(lambda1*h);
p2 = exp(lambda2*h);

%define the feedback gain for the discrete time system (3) as K1
K1 = place(A, B, [p1,p2]);

%define the feedback gain for the delayed discrete time system (4) as K2
K2 = place(Aa, Ba, [p1 p2 0]);  %exp(real(lambda1)*10*h)

%plot the step response of the systems in one figure. Your figure should
%have labels and legend.
[step1, stept] = step( ss(A-B*K1, B,  C, 0, h) , 3);
step2 = step( ss(Aa-Ba*[K1 0], Ba, Ca, 0, h) , 3);
step3 = step( ss(Aa-Ba*K2, Ba, Ca, 0, h) , 3);

close all
figure('Color','white'), hold on, grid on;
stairs(stept, step1, '-', 'LineWidth', 2);
stairs(stept, step2, '--', 'LineWidth', 2);
stairs(stept, step3, '--', 'LineWidth', 2);
title 'Step in reference r(k)', xlabel 'Time [s]', ylabel 'Step-response amplitude \theta(k)'
legend({'Close-loop (2) - K1','Close-loop (4) - K1','Close-loop (4) - K2'})
% fp.savefig('Kplace');


%% Steady State
ys=pi/6;
% plot the system output as explained in Question #7. Your figure should
%have labels and legend.

x = sym('x',[3,1]);
syms u
eq = [x == Aa*x + Ba*u;
      ys == Ca*x];
answ = solve(eq);
xs = double([answ.x1; answ.x2; answ.x3]);
us = double(answ.u);

x0 = zeros(3,1);
dx = [];
dx(:,1) = x0 - xs;
t=0:h:3;
for i=2:numel(t)
    du = -K2*dx(:,i-1);
    dx(:,i) = Aa*dx(:,i-1) + Ba*du;
end
y = Ca*(dx + xs);

figure('Color','white'), hold on, grid on;
stairs(t,y,'-', 'LineWidth', 2);
title 'Step in reference r(k)', xlabel 'Time [s]', ylabel 'Step-response amplitude \theta(k)'
legend({'Close-loop (4) - K2'})
% fp.savefig('Rtracking');


%% disturbance
Bd=[0;1;0];

% define Ae, Be, Ce, and De as asked in Question #8
nd = size(Bd,2);
n  = size(Aa,1);
Ae = [Aa Bd; 0*(Aa*Bd)' eye(nd)];
Be = [Ba; zeros(nd,1)];
Ce = [Ca zeros(nd,1)];
De = 0;

%define the rank of the controllability and observability matrices as
sys6_c = rank(ctrb(Ae,Be));
eig(Ae);
rank([1.0733*eye(4)-Ae Be])  % unstable pole is controllable
rank([1*eye(4)-Ae Be])       % marginally stable pole is not controllable
% => system is not stabilizable

sys6_o = rank(obsv(Ae,Ce));



%% Feedback gain
%define the controller gain as K3 according to Question #9
K3 = place(Aa,[Ba Bd],[p1 p2 .999]);
eig(Aa-[Ba Bd]*K3)

step(ss(Aa-[Ba Bd]*K3, [Ba Bd], Ca, 0, h))


%% Observer
%define L as the observer gain according to Question #10

L = place(Ae', Ce', [0.1, 0.2, 0.3, 0.4])';






