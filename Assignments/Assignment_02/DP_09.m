function [K0,P0]=DP_XX(A,B,N,Q,R,Pf)
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% K0 is the controller gain when u(0)=K0x(0)
%% P0 describes the final cost as VN=(x0^T)*P0*x0 
   
    P = cell(N+1,1);
    P{N+1} = Pf;        % P0=P{1}, Pf=PN=P{N+1}
    for i=N+1:-1:2
        P{i-1} = Q + A'*P{i}*A - A'*P{i}*B*inv(R + B'*P{i}*B)*B'*P{i}*A;
    end
    K0 = inv(R+B'*P{2}*B)*B'*P{2}*A;
    P0 = P{1};
end

