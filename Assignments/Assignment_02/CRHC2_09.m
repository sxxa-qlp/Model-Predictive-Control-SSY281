function [Z,VN]=CRHC2_09(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)
% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
% Q, R, and Pf are the gains in the cost function
% N is the length of the horizon
% Z is the vector of optimal variables and VN is the cost function 
% F1, G1, h1, F2, G2, h2 are constraint matrices
% x0 is the initial condition

    % Z = u
    
    % Cost function defined in equation (22)
    % vector x in the constraints is replaced by equation (21)
    % Constraints are given by equation (41)

%% Batch parameters

    gamma = kron(eye(N),B);
    omega = A;
    for i=1:N-1
        gamma = gamma + kron(diag(ones(N-i,1),-i),A^i*B);
        omega = [omega; A^(i+1)];
    end
    Qb = blkdiag( kron(eye(N-1),Q), Pf );
    Rb = kron(eye(N),R);
    
%% Define LQ quadratic minimization equation (0.5*x'*H*x + f'*x)
    
    H = 2*(gamma'*Qb*gamma + Rb);
    f = (2*x0'*omega'*Qb*gamma)';

%% Define inequalities constraints
    
    % F2*x + G2*u < h2  => F2*(omega*x0+gamma*u) + G2*u < h2 
    %                   => (F2*gamma+G2)*u < h2-F2*omega*x0
    Ain = F2*gamma+G2;
    bin = h2-F2*omega*x0;
    
%% Define system dynamics + equalities constraints

    Aeq = F1*gamma+G1;
    beq = h1-F1*omega*x0;

%% Solve
    options = optimoptions('quadprog','Display','none');
    [Z,VN,~] = quadprog(H,f, Ain,bin, Aeq,beq, [],[], [],options);
end
