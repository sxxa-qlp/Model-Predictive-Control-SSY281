function P=Pre_09(A,B,S,U)
% A and B are the system matrices x^+=Ax+Bu
% S is the polytope for set S
% U is the polytope for feasible inputs
% P is the polytope Pre(S)

    Ain = [S.A*A    S.A*B
           zeros(size(U.A,1),size(S.A*A,2))     U.A];
    bin = [S.b; U.b];

    Pext = Polyhedron('A',Ain, 'b',bin);

    P = projection(Pext,[1:length(A)]);

end