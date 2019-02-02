function [K0,P0]=BS_09(A,B,N,Q,R,Pf)
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% K0 is the controller gain when u(0)=K0x
%% P0 describes the final cost as VN=(x0^T)*P0*x0 

    gamma = kron(eye(N),B);
    omega = A;
    for i=1:N-1
        gamma = gamma + kron(diag(ones(N-i,1),-i),A^i*B);
        omega = [omega; A^(i+1)];
    end
    
    Qb = blkdiag( kron(eye(N-1),Q), Pf );
    Rb = kron(eye(N),R);
    
    P0 = (Q + omega'*Qb*omega - omega'*Qb*gamma*inv(gamma'*Qb*gamma + Rb)*gamma'*Qb*omega);
    
    K = (gamma'*Qb*gamma + Rb)\gamma'*Qb*omega;
    K0 = K(1:size(B,2),:);
    
end
