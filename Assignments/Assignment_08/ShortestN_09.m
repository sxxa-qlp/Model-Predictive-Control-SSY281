function [Z,exitflag]=ShortestN_09(A,B,N,Q,R,Pf,x_ub,u_ub,x0)
% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
% Q, R, and Pf are the gains in the cost function
% N is the length of the horizon
% Z is the vector of optimal variables 
% F1, G1, h1, F2, G2, h2 are constraint matrices
% x0 is the initial condition
% x_ub is the upper bound for absolute value of x elements
% u_ub is the upper bound for absolute value of u elements
% exitflag shows if the quadprog have a solution or not; it is one of quadprog outputs


    m = size(B,2);  % inputs
    n = size(A,1);  % states
    
    % F1*x + G1*u =  h1
    F1_i = zeros(1,n);
    G1_i = zeros(1,m);
    h1_i = 0;
    
    % F3*xf =  h3
    F3_i = eye(n);
    G3_i = zeros(n,m);
    h3_i = zeros(n,1);

    % F2*x + G2*u <= h2
    F2_i = [eye(n)
            -eye(n)
            zeros(m,n)
            zeros(m,n)];
    G2_i = [zeros(n,m)
            zeros(n,m)
            eye(m)
            -eye(m)];
    h2_i = [x_ub
            x_ub
            u_ub
            u_ub];

    F1 = blkdiag( kron(eye(N-1),F1_i), F3_i );
    G1 = blkdiag( kron(eye(N-1),G1_i), G3_i );
    h1 = [kron(ones(N-1,1),h1_i); h3_i ];
    
    F2 = kron(eye(N),F2_i);
    G2 = kron(eye(N),G2_i);
    h2 = kron(ones(N,1),h2_i);
    

    [Z,~,exitflag] = CRHCX_09(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0);
    
    function [Z,VN,exitflag]=CRHCX_09(A,B,N,Q,R,Pf,F1,G1,h1,F2,G2,h2,x0)
    % A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
    % Q, R, and Pf are the gains in the cost function
    % N is the length of the horizon
    % Z is the vector of optimal variables and VN is the cost function 
    % F1, G1, h1, F2, G2, h2 are constraint matrices
    % x0 is the initial condition

        % F1*x + G1*u =  h1
        % F2*x + G2*u <= h2
        % Z = [x ; u]

        % This functions is programmed following algorithm 45 (page 43)

    % Batch parameters

        gamma = kron(eye(N),B);
        omega = A;
        for i=1:N-1
            gamma = gamma + kron(diag(ones(N-i,1),-i),A^i*B);
            omega = [omega; A^(i+1)];
        end
        Qb = blkdiag( kron(eye(N-1),Q), Pf );
        Rb = kron(eye(N),R);

    % Define LQ quadratic minimization equation

        % min x0'*Q*x0 + x'*Qb*x + u'*Rb*u  -> 0.5*z'*H*z + f'*z
        % => H = 2*blkdiag(Qb,Rb)
        H = 2*blkdiag(Qb,Rb);
        f = zeros(size(H,1),1);

    % Define inequalities constraints

        Ain = [F2,G2];
        bin = h2;

    % Define system dynamics + equalities constraints

        % System dynamics: x = omega*x0 + gamma*u
        Aeq = [kron(eye(N),eye(size(A))), -gamma;
               F1,G1];
        beq = [omega*x0;
               h1];

    % Solve
        options = optimoptions('quadprog','Display','none');
        [Z,VN,exitflag] = quadprog(H,f, Ain,bin, Aeq,beq, [],[], [],options);

    end
    
end







