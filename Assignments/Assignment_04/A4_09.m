clear;clc;
%% Q1-Q3: Explain and answer in the report
%% Q4: Fill in N1_XX.m and Ninf_XX.m in which XX is your group ID.

    A = [10 1; 3 1];
    b = [5 ; 2];
    [x,fval]=N1_09(A,b)
    
    A = [1 0; 0 2];
    b = [1 ; 5];
    [x,fval]=Ninf_09(A,b)

%% Q5: Provide your results and explanations in the report. Your code should come here to support 
% your results in the report

H = eye(4);
f = zeros(4,1);

% x1 - u0 = 1
% x2 - 0.5x1 -u1 = 0
Aeq = [1 0 -1 0;
       -0.5 1 0 -1];
beq = [1;0];

Aleq= [eye(4);
       -eye(4)];
bleq= [5 1 2 2   -2.5 1 2 2]';

options = optimoptions('quadprog','Display','iter');
[x,fval,exitflag,output,lambda] = quadprog(H,f,Aleq,bleq,Aeq,beq,[],[],[],options);

x
fval
lambda.ineqlin


x + Aleq.'*lambda.ineqlin + Aeq'*lambda.eqlin





%% Remove the x1 lower bound
Aeq = [1 0 -1 0;
       -0.5 1 0 -1];
beq = [1;0];

Aleq= [eye(4);
       -eye(4)];
bleq= [5 1 2 2   -2.5 1 2 2]';
Aleq = Aleq([1:4 6:8],:);
bleq = bleq([1:4 6:8]);

options = optimoptions('quadprog','Display','iter');
[x,fval,exitflag,output,lambda] = quadprog(H,f,Aleq,bleq,Aeq,beq,[],[],[],options)

x
fval
lambda.ineqlin

%% Remove the x1 upper bound
Aeq = [1 0 -1 0;
       -0.5 1 0 -1];
beq = [1;0];

Aleq= [eye(4);
       -eye(4)];
bleq= [5 1 2 2   -2.5 1 2 2]';
Aleq = Aleq([2:8],:);
bleq = bleq([2:8]);

options = optimoptions('quadprog','Display','iter');
[x,fval,exitflag,output,lambda] = quadprog(H,f,Aleq,bleq,Aeq,beq,[],[],[],options)

x
fval
lambda.ineqlin
