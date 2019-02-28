function [Z,exitflag]=RHCXf_09(A,B,N,Q,R,Pf,x_ub,u_ub,Xf,x0)
%% A and B are the system matrices when x(k+1)=Ax(k)+Bu(k)
%% Q, R, and Pf are the gains in the cost function
%% N is the length of the horizon
%% Z is the vector of optimal variables 
%% x0 is the initial condition
%% x_ub is the upper bound for absolute value of x elements
%% u_ub is the upper bound for absolute value of u elements
%% exitflag shows if the quadprog have a solution or not; it is one of quadprog outputs
%% Xf is a polytope that shows the terminal set


end
