function [Z,exitflag]=ShortestN_09(A,B,N,Q,R,Pf,x_ub,u_ub,x0)
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% Z is the vector of optimal variables 
%% F1, G1, h1, F2, G2, h2 are constraint matrices
%% x0 is the initial condition
%% x_ub is the upper bound for absolute value of x elements
%% u_ub is the upper bound for absolute value of u elements
%% exitflag shows if the quadprog have a solution or not; it is one of quadprog outputs


end
