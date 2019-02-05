function [Z,VN]=CRHC1_09(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)
% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
% Q, R, and Pf are the gains in the cost function
% N is the length of the horizon
% Z is the vector of optimal variables and VN is the cost function 
% F1, G1, h1, F2, G2, h2 are constraint matrices
% x0 is the initial condition

    % F1*x + G1*u =  h1
    % F2*x + G2*u <= h2
    % Z = [x ; u]
    
    % min x0'*Q*x0 + x'*Qb*x + u'*Rb*u  -> 0.5*z'*H*z + f'*z
    % => H = 2*blkdiag(Qb,Rb)

%% Batch parameters

    gamma = kron(eye(N),B);
    omega = A;
    for i=1:N-1
        gamma = gamma + kron(diag(ones(N-i,1),-i),A^i*B);
        omega = [omega; A^(i+1)];
    end
    Qb = blkdiag( kron(eye(N-1),Q), Pf );
    Rb = kron(eye(N),R);
    
%% Define LQ quadratic minimization equation
    
    H = 2*blkdiag(Qb,Rb);
    f = zeros(size(H,1),1);

%% Define inequalities constraints
    Aleq = [F2,G2];
    bleq = h2;
    
%% Define system dynamics + equalities constraints
    
    % System dynamics: x = omega*x0 + gamma*u
    Aeq = [kron(eye(N),eye(size(A))), -gamma;
           F1,G1];
    beq = [omega*x0;
           h1];

%% Solve
    options = optimoptions('quadprog','Display','none');
    [Z,VN,exitflag] = quadprog(H,f, Aleq,bleq, Aeq,beq, [],[], [],options);
    
end
