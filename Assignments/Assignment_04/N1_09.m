function [x,fval]=N1_09(A,b)
% This function finds x to minimize the first norm
% The inputs are Matrix a and vector b
% The outputs are the optimal x and the norm value (fval)
% Do not change the inputs and outputs!

    n = length(b);
    f = [zeros(n,1);
         ones(n,1)];
    Aeq = [ A -eye(n);
           -A -eye(n)];
    beq = [ b;
           -b];

    options = optimoptions('linprog','Algorithm','interior-point','Display','iter');
    [z,fval] = linprog(f,[],[],Aeq,beq,[],[],options);
    x = z(1:n);
end

