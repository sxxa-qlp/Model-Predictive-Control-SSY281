clear all; close all; clc;

A = [0.5 1; -1 0.5];
B = [0;1];
C = eye(2);
Q = eye(2);
ysp = [1;-1];

ST = [eye(2)-A -B;
      C         0*C*B];
  
m = size(B,2);
p = size(C,1);
  
% More outputs than inputs case (p>m)

% (C*xs-ysp)'*Q*(C*xs-ysp) = x'*(C'*Q*C)*x - xs'*C*Q*ysp - ysp'*Q*C*xs + ysp'*Q*ysp
H = blkdiag(C'*Q*C, zeros(m));
f = [ -2*(ysp'*Q*C)'; zeros(m,1)];
Aleq = [];
bleq = [];
Aeq = [eye(2)-A -B];
beq = zeros(2,1);

options = optimoptions('quadprog','Display','none');
[xs_us,VN,exitflag] = quadprog(H,f, Aleq,bleq, Aeq,beq, [],[], [],options)


ST*xs_us - [zeros(2,1); ysp]