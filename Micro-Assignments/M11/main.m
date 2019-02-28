clear all; close all; clc;

B = [1; 0];
C = [0, 1];
syms a11 a12 a22
A = [a11, a12; a12, a22];


resp = solve( [C*B , C*A*B-0.5, C*A^2*B-1] )

A = [resp.a11, resp.a12; resp.a12, resp.a22]
B
C