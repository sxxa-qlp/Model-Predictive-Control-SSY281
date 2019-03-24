%  Write a code that does the simulation and plots the system output and
%  input with respect to time.
% Your plots should have meaningful labels and simulates the system after
% the convergence to the final set as well to shows the system remains in
% it. Explain the concept that you have used to design the controller in
% your report, provide the simulation outcomes, and explain your
% observations.


%% Model parameters

clear;close all;clc;yalmip('clear');

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

% Continuous time state-space matrices
Ac = [0 1 0 0;
     -kth/JL -betaL/JL kth/(rho*JL) 0;
     0 0 0 1;
     kth/(rho*JM) 0 -kth/(rho^2*JM) -(betaM+kT^2/R)/JM];
Bc = [0 0 0 kT/(R*JM)].';
Cc = [kth 0 -kth/rho 0];

% Sampling time
dt = 0.06;
% dt = 0.05;
dt = min(dt, pi/max(damp(Ac)));    % [s]

% Convert to discrete time
sys=ss(Ac,Bc,Cc,0);
sysd=c2d(sys,dt);
A = sysd.A;
B = sysd.B;
C = sysd.C;

% size of problem to be solved
nx = size(A,2);
nu = size(B,2);

% input and output constraints (lower and upper bound)
T_b = [-157 157];
V_b = [-200 200];


%% Create MPC model
plotP = @(Pol) plot(projection(Pol,[2 4]));

% MPC parameters
Q=1e-1; 
R=1e-2;

% initial state
x0 = [0 2.5 0 75]';

X = Polyhedron('A',[C;-C],'b',[T_b(2); -T_b(1)]);
U = Polyhedron('lb',V_b(1),'ub',V_b(2));

% Xf = Polyhedron( [-0.1<=xsdp([2,4])<=0.1 ;-Inf<=xsdp([1,3])<=Inf ] ); %

% for p1=0.00:0.0005:0.03
err = false;

p1 = 0.023;%0.024;
p2 = Inf;
xsdp = sdpvar(nx, 1);
Xf = Polyhedron( [-p1<=xsdp([2,4])<=p1;-p2<=xsdp([1,3])<=p2] ); %

% define model
model = LTISystem('A', A, 'B', B, 'C', C);
model.x.with('setConstraint');
model.x.setConstraint = X;
model.u.with('setConstraint');
model.u.setConstraint = U;
model.x.with('terminalSet');
model.x.terminalSet = Xf;
% model.x.penalty = QuadFunction(Q);
% model.u.penalty = QuadFunction(R);
% mintime = EMinTimeController(model)

K = Xf;
isin = false; 
i = 0;
while ~isin
    i = i+1
    K(i+1) = model.reachableSet('X', K(i), 'U', U, 'N', 1, 'direction', 'backward');
%     K(i+1) = K(i+1).intersect(X);
    K(i+1).irredundantHRep();
    isin =  K(i+1).contains(x0);% all(K(end).A*x0-K(end).b <= 0);
    
    % optimization variables
    x = sdpvar(nx, 1);
    u0 = sdpvar(nu, 1);
    
    % cost function
    J = u0'*R*u0 + (A*x + B*u0)'*C'*Q*C*(A*x + B*u0);

    % constraints
    con = [ T_b(1) <= C*x <= T_b(2);                % x0 \in X
            K(i+1).A * x <= K(i+1).b;               % x0 \in Pre^{i}(Xf)
            V_b(1) <= u0 <= V_b(2);                 % u0 \in U
            T_b(1) <= C*(A*x + B*u0) <= T_b(2);     % x1 \in X
            K(i).A*(A*x + B*u0) <= K(i).b;          % x1 \in Pre^{i-1}(Xf)
            ];
        
%     figure; plot(projection(Polyhedron(con),[2 4]))
%     figure; plot(projection(Polyhedron(con),[1 3]))
        
    [sol,diagnostics,aux,Valuefunction,Optimal_z] = solvemp(con, J, [], x, u0);
    mpsol{i} = sol{1};
    Optimalz{i} = Optimal_z;
    
    plp = Opt(con, J, x, u0);%     plp = Opt(con, J, x, u0);
    mpsol2{i} = plp.solve();

    if isempty(sol{1})
        error('Empty parametric solution');
        return;
%         err = true;
%         break;
    end
end

% if ~err
%     break;
% end
% end

%% Simulate


% % % % figure; hold on;
% % % % for i=1:numel(K)
% % % %     plot(projection(K(i),[2 4]),'alpha',0.3, 'Color',fp.getColor(i))
% % % % end
% % % % xlim([-50 50])
% % % % ylim([-100 100])

tf = 50;
xv = nan(nx,tf+1);
u0 = nan(1,tf);
xv(:,1) = x0;
x0

try
    for ts=1:tf

% % % %         % find controller partition    
% % % %         ipart = 1;
% % % %         while ~K(ipart+1).contains(xv(:,ts))  % K(ipart+1).A *  xv(:,ts) - K(ipart+1).b
% % % %             ipart = ipart+1;
% % % %         end
% % % %         part(ts) = ipart
% % % % 
% % % %         % find controller region
% % % %         [isin,ireg] = isinside(mpsol{ipart}.Pn, xv(:,ts));

        ipart = 0;
        isin = false;
        while ~isin
            ipart = ipart+1;
            [isin,ireg] = isinside(mpsol{ipart}.Pn, xv(:,ts));
        end
        part(ts) = ipart
        
% % % %         for ipart=1:numel(mpsol2)
% % % %             for ireg =1:numel(mpsol2{ipart}.xopt.Set)
% % % %                 if mpsol2{ipart}.xopt.Set(ireg).contains( xv(:,ts) )
% % % %                     u0(ts) = mpsol2{ipart}.xopt.feval( xv(:,ts) ,'primal')
% % % %                     break;
% % % %                 end
% % % %             end
% % % %         end
        
% % % %         mpsol2{3}.xopt.contains( xv(:,ts) )
% % % %         a = xv(:,ts)
% % % %         ipart

% % %         u0(ts) = mpsol{ipart}.Fi{ireg}*xv(:,ts) + mpsol{ipart}.Gi{ireg}
        
        assign(x,xv(:,ts));
        value(Optimalz{ipart})

        if abs(u0(ts)) > 201
            error('input out of bounds');
        end

        % simulate - update states
        xv(:,ts+1) = A*xv(:,ts) + B*u0(ts);
        xv(:,ts+1)
    end
catch err
    fprintf('%s\n',err.message)
end
% % % % plot(xv(2,:)',xv(4,:)','*-', 'LineWidth',3)


figure('Color','white','Position',[283   85  813  783]);
ax(1) = subplot(4,1,1);
stairs(1:tf+1, xv([2,4],:)', 'LineWidth',3)
ylabel 'axle velocities'
ax(2) = subplot(4,1,2);
stairs(1:tf+1, (C*xv)', 'LineWidth',3)
ylabel 'Output - Torsional torque'
ylim(T_b)
ax(3) = subplot(4,1,3);
stairs(1:numel(u0), u0, 'LineWidth',3)
ylabel 'Input - DC Voltage'
ylim(V_b)
ax(4) = subplot(4,1,4);
stairs(1:numel(part), part, 'LineWidth',3)
ylabel 'Partition'
% ylim(V_b)
linkaxes(ax,'x')
xlim([1 tf+1])








