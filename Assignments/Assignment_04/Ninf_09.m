function [x,fval]=Ninf_09(A,b)
% This function finds x to minimize the infinity norm
% The inputs are Matrix a and vector b
% The outputs are the optimal x and the norm value (fval)
% Do not change the inputs and outputs!

    n = length(b);
    f = [zeros(n,1);
         1];
    Aeq = [ A  -ones(n,1);
           -A  -ones(n,1)];
    beq = [ b;
           -b];

    options = optimoptions('linprog','Algorithm','interior-point','Display','iter');
    [z,fval] = linprog(f,[],[],Aeq,beq,[],[],options);
    x = z(1:n);
end