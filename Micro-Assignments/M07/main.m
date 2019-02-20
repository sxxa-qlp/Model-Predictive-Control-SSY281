clear all; close all; clc;


%% Question 1

syms lbd mu x1 x2
L = x1^2 + 2*x2^2 + lbd*(x1-2*x2-1) + mu*(x1-2)
xopt = solve( jacobian(L,[x1,x2]).' , [x1 x2]);
q = simplify(subs(L, [x1 x2], [xopt.x1 xopt.x2]))

%% Question 2

L = x1 + x2 + lbd*(x1 + 2*x2 -5) + mu*(x1-1)

jacobian(L,[x1,x2])


%% Question 3

% Primal formulation
H = 2*[1 0; 0 2];
f = [0 ; 0];
A = [1 0];
b = 2;
Aeq = [1 -2];
beq = 1;
[varprimal,p0,exitflag,output] = quadprog(H,f,A,b,Aeq,beq);

% Dual formulation
H = 2*[-3/4 -1/4; -1/4 -1/4];
f = [-1 ; -2];
A = [0 -1];
b = 0;
% -H and -f in order to convert the problem to minimization
[vardual,d0,exitflag,output] = quadprog(-H,-f,A,b);

p0
varprimal

d0
vardual
xd = [-vardual(1)/2 - vardual(2)/2 ; vardual(1)/2]



