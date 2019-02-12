clear;
close all;
clc;

%% Question 1

A=diag([0.5 0.6 0.5 0.6]);
B=[diag([0.5 0.4]);diag([0.25 0.6])];
C=[1 1 0 0;0 0 1 1];
z_sp=[1;-1];

% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report

% Number of outputs and inputs are the same (p=m)
p = size(C,1);  % outputs
m = size(B,2);  % inputs
n = size(A,1);  % states

xs_us = [eye(n)-A -B; C C*B*0] \ [zeros(n,1) ; z_sp]

xs1 = xs_us(1:n)
us1 = xs_us(end-m+1:end)

% Test steady-state conditions
[eye(n)-A -B; C C*B*0]*xs_us - [zeros(n,1) ; z_sp]

%% Question 2

A=diag([0.5 0.6 0.5 0.6]);
B=[0.5;0;0.25;0];
C=[1 1 0 0;0 0 1 1];
z_sp=[1;-1];
% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report

% There are more outputs than inputs (p > m)
p = size(C,1);  % outputs
m = size(B,2);  % inputs
n = size(A,1);  % states

% min   (C*xs-z_sp)'*Q*(C*xs-z_sp) = xs'*(C'*Q*C)*xs + (-2*z_sp'*Q*C)*xs + z_sp'*Q*z_sp
Q = eye(p);
H = 2* blkdiag( C'*Q*C , zeros(m) );
f = [ (-2*z_sp'*Q*C)' ; zeros(m,1) ];

Aeq = [eye(n)-A,-B];
beq = zeros(n,1);

options = optimoptions('quadprog','Display','iter');
xs_us = quadprog(H ,f,[],[],Aeq,beq,[],[],[],options)

xs2 = xs_us(1:n)
us2 = xs_us(end-m+1:end)

% Test steady-state conditions
[eye(n)-A -B; C C*B*0]*xs_us - [zeros(n,1) ; z_sp]


%% Question 3
A=diag([0.5 0.6 0.5 0.6]);
B=[diag([0.5 0.4]);diag([0.25 0.6])];
C=[1 1 0 0];
z_sp=1;

% calculate the steady state (xs,us) and define them as follows; Do not change
% the names of variables; Report these values in the report

% There are inputs than outputs (p < m)
p = size(C,1);  % outputs
m = size(B,2);  % inputs
n = size(A,1);  % states


% min   (us-usp)'*Rs*(us-usp)
H = 2* blkdiag( zeros(n) , eye(m) );
f = [ zeros(n,1) ; zeros(m,1) ];

Aeq = [eye(n)-A -B; C 0*C*B];
beq = [zeros(n,1) ; z_sp];

options = optimoptions('quadprog','Display','iter');
xs_us = quadprog(H ,f,[],[],Aeq,beq,[],[],[],options)


xs3 = xs_us(1:n)
us3 = xs_us(end-m+1:end)

% Test steady-state conditions
[eye(n)-A -B; C C*B*0]*xs_us - [zeros(n,1) ; z_sp]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% second part

tf=50;                
%==========================================================================
% Process model
%==========================================================================

h = 1; %sampling time in minutes

A = [ 0.2681   -0.00338   -0.00728;
      9.703    0.3279   -25.44;
         0         0       1   ];
B = [ -0.00537  0.1655;
       1.297   97.91 ;
       0       -6.637];
C = [ 1 0 0;
      0 1 0;
      0 0 1];
Bp = [-0.1175;
      69.74;
       6.637 ];
   
n = size(A,1); % n is the dimension of the state
m = size(B,2); % m is the dimension of the control signal
p = size(C,1); % p is the dimension of the measured output

d=0.01*[zeros(1*tf/5,1);ones(4*tf/5,1)]; %unmeasured disturbance trajectory

x0 = [0.01;1;0.1]; % initial condition of system's state

%==========================================================================
% Observer model
%==========================================================================
%% choose one case, i.e. a, b, or c and then write the code for that case! for the other ones
% you just need to change the example case!
example = 'c';
switch example
    case 'a'
        nd = 2;
        Bd = zeros(n,nd);
        Cd = [1 0;0 0; 0 1]; 
    case 'b'
        nd=3;
        Bd = zeros(n,nd); 
        Cd = [1 0 0;0 0 1;0 1 0];
    case 'c'
        nd=3;
        %Bd = [0 0 0.1655;0 0 97.91; 0 0 -6.637]; 
        Bd = [zeros(3,2) Bp];
        Cd = [1 0 0;0 0 0;0 1 0];
end

%% Question 4
%Augment the model with constant disturbances; check the detectability for case "a", "b", and "c" and
%report the detectability of each one in the report

% define Ae, Be, and Ce which are the matrices for the augmented system. No
% need to report these in the report

Ae = [A Bd; zeros(nd,n) eye(nd)]
Be = [B; zeros(nd,m)]
Ce = [C Cd]

% Detectability test - equation (13)
eig(Ae);
rank(ctrb(A,B));
rank(obsv(A,C)) == n                    % svd(obsv(A,C))
rank([eye(n)-A -Bd; C Cd]) == n+nd      % same of testing svd(obsv(Ae,Ce))


%% Question 5
% Calculate Kalman filter gain and name it Le; no need to report this in
% the report

Q = eye(n+nd);
R = eye(p);
[P,lambda,L] = dare(Ae',Ce',Q,R);

abs(lambda)

% Kalman Filter - filtering case
Le = Ae\L'                         % Same result as P*Ce' / (Ce*P*Ce'+R)   

%% Question 6
%Target Selector
H = [1 0 0;0 0 1];

% Matrices for steady state target calculation
Mss = [eye(n)-A -B; H*C 0*H*C*B] \ [Bd ; -H*Cd];

%note that Mss is defined by [xs;us]=Mss*d and you will use it later on; no need to report this in
% the report

%% Question 7
%==========================================================================
% Setup MPC controller
%==========================================================================

ysp = zeros(tf,3);         % setpoint trajectory

N=10;                   % prediction horizon
M=3;                    % control horizon

Q = diag([1 0.001 1]);  % state penalty
Pf = Q;                 % terminal state penalty
R = 0.01*eye(m);        % control penalty
%==========================================================================
% Simulation
%==========================================================================
    
%=============================
% Init variables
%=============================
xe_h(:,1) = [zeros(n,1) ; zeros(nd,1)];
x(:,1)  = x0;

% Simulation
    for k = 1:tf
        
        %=============================
        % Calculate steady state target
        %=============================
        d_h = xe_h(end-nd+1:end, k);
        xs_us = Mss*d_h;
        xs = xs_us(1:n);
        us = xs_us(end-m+1:end);
        
        %=============================
        % Solve the QP
        %=============================
        x_h = xe_h(1:n, k);
        dx = x_h - xs;

        [du,~] = CRHC2_09(A,B,N,M,Q,R,Pf,[],[],[],[],[],[],dx);
        u(:,k) = du(1:m) + us;       
%         [u0,z]=RHC(A,B,Q,R,Pf,N,M,dx);
%         u(:,k) = u0 + us;
        
        %=============================
        % Update the observer state
        %=============================
        % Measure
        y(:,k) = C*x(:,k);
        % Correction
        xe_h_k = xe_h(:,k) + Le*(y(:,k) - Ce*xe_h(:,k));
        % Update
        xe_h(:,k+1) = Ae*xe_h_k + Be*u(:,k);
        
        %=============================
        % Update the process state
        %=============================
        x(:,k+1) = A*x(:,k) + B*u(:,k) + Bp*d(k);

   end % simulation loop
 

%==========================================================================
% Plot results
%==========================================================================
%     close all
    
    figure('Color','white', 'Position', [250.6000   44.2000  779.2000  717.6000])
    
    subplot(3,2,1)
    stairs(x(1:n,:)', 'LineWidth',2)
    ylabel 'x', grid on
    legend({'x1','x2','x3'});
    
    subplot(3,2,2)
    stairs(xe_h(1:n,:)', 'LineWidth',2)
    ylabel 'x hat', grid on
    
    subplot(3,2,3)
    stairs(x(1:n,:)'-xe_h(1:n,:)', 'LineWidth',2)
    ylabel 'estimation error x', grid on
    
    subplot(3,2,4)
    stairs(xe_h(end-nd+1:end,:)', 'LineWidth',2)
    ylabel 'd hat', grid on
    legend({'d1','d2','d3'});
    
    subplot(3,2,5)
    stairs( (H*(C*x(:,1:tf)-ysp'))', 'LineWidth',2)
    ylabel 'error zsp', grid on
    legend({'zsp_1','zsp_2','zsp_3'});
    
    subplot(3,2,6)
    stairs(u', 'LineWidth',2)
    ylabel 'u', grid on
    legend({'u_1','u_2','u_3'});
    
    sgtitle(['System ' example])
    
%     fp.savefig(['system-',example]);
    
   % plot the states, the state estimations, and the input and report them
   % in the report.
   
%% LQ solver

function [Z,VN]=CRHC2_09(A,B,N,M,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)
% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
% Q, R, and Pf are the gains in the cost function
% N is the length of the horizon
% Z is the vector of optimal variables and VN is the cost function 
% F1, G1, h1, F2, G2, h2 are constraint matrices
% x0 is the initial condition


% Batch parameters

    gamma = kron(eye(N),B);
    omega = A;
    for i=1:N-1
        gamma = gamma + kron(diag(ones(N-i,1),-i),A^i*B);
        omega = [omega; A^(i+1)];
    end
    Qb = blkdiag( kron(eye(N-1),Q), Pf );
    Rb = kron(eye(N),R);
    
    % Define LQ quadratic minimization equation (0.5*x'*H*x + f'*x)    
    H = 2*(gamma'*Qb*gamma + Rb);
    f = (2*x0'*omega'*Qb*gamma)';

    % Define inequalities constraints
    % F2*x + G2*u < h2  => F2*(omega*x0+gamma*u) + G2*u < h2 
    %                   => (F2*gamma+G2)*u < h2-F2*omega*x0
    if ~isempty(F2)
        Ain = F2*gamma+G2;
        bin = h2-F2*omega*x0;
    else
        Ain = []; bin= [];
    end
    
    % Define system dynamics + equalities constraints
    % F1*x + G1*u = h1
    % After control horizon, the inputs is kept constant
    Aeq = kron([zeros(N-M,M-1) -1*ones(N-M,1) eye(N-M)], eye(size(B,2)));
    beq = kron( zeros(N-M,1), zeros(size(B,2),1));
    if ~isempty(F1)
        Aeq = [ Aeq ; F1*gamma+G1];
        beq = [ beq ; h1-F1*omega*x0];
    end

    % Solve
    options = optimoptions('quadprog','Display','none');
    [Z,VN,~] = quadprog(H,f, Ain,bin, Aeq,beq, [],[], [],options);
end

