clear;clc;close all;
%%You are supposed to fill this template. Remeber not to change variable
%%names!
%% Discrete Model
Ac=[0 1;0.5 0];Bc=[0;1];Cc=[1 0];Dc=0;

%define the sampling time here
wn = damp(Ac);
h = [1 0] * (2*pi./(2*wn));    % [s]

%Use given parameters and find A, B, and C in Question #1
sys=ss(Ac,Bc,Cc,Dc);
sysd=c2d(sys,h);

A = sysd.A;
B = sysd.B;
C = sysd.C; 

%% Delayed model
% define Aa, Ba, Ca, Da according to Question #2

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


% Question 5
minreal(Ca*inv(tf('z')*eye(3) - Aa)*Ba)




%% Controller design
lambda1=-4+6*1i;
lambda2=-4-6*1i;

%calculate desired poles for the discrete time system (3) and define them as p1 and
%p2 as it is asked in Question #6
%p1=
%p2=

%define the feedback gain for the discrete time system (3) as K1
%K1=

%define the feedback gain for the delayed discrete time system (4) as K2
%K2=


%plot the step response of the systems in one figure. Your figure should
%have labels and legend.


%% Steady State
ys=pi/6;
% plot the system output as explained in Question #7. Your figure should
%have labels and legend.


%% disturbance
Bd=[0;1;0];
% define Ae, Be, Ce, and De as asked in Question #8
%Ae=
%Be=
%Ce=
%De=

%define the rank of the controllability and observability matrices as

%sys6_c=
%sys6_o=


%% Feedback gain
%define the controller gain as K3 according to Question #9
%K3=




%% Observer
%define L as the observer gain according to Question #10
%L=

