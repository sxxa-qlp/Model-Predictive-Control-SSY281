function [u0,z]=RHC(A,B,Q,R,Pf,N,M,x0)
% This is an example of the RHC when there is no constraint.

    % N prediction horizon
    % M control horizon
    % Q state penalty
    % Pf terminal state penalty
    % R control penalty
    
    n=length(x0);
    m=length(B(1,:));

    H_Q = kron(eye(N-1),Q); % This is for x(k+1) to x(k+N-1);
    H_last = Pf;            % This is only for x(k+N);
    H_R = kron(eye(M),R);   % This is for u(k) to u(k+N-1)
    HM = blkdiag(H_Q, H_last, H_R );
    
    Aeq_x = [zeros(n,n*N);kron(eye(N-1),A),zeros(n*(N-1),n)]-eye(n*N);
    Aeq_u = kron(eye(M),B);  
    Aeq_ub= repmat(Aeq_u((M-1)*n+1:M*n,:),N-M,1);
    Aeq_u = [Aeq_u;Aeq_ub];
    Aeq   = [Aeq_x, Aeq_u];
    beq = [-A*x0;zeros(n*(N-1),1)];
    
    Ain = [];
    Bin = [];
    
    z = quadprog(HM,[],Ain,Bin,Aeq,beq); % z constains optimal x and u, i.e. z=[x(1);x(2);...;x(N);u(0);...;u(M-1)];
    u0=z(n*N+1:n*N+m);    
    end