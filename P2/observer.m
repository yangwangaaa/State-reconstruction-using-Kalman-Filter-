clear all;
close all;
clc;

%% The system matrices
a = [ -0.0000 1.0000 -0.0000 -0.0000;
    -0.0000 -0.5004 -0.0022 0.0064;
    -30.8405 -66.2409 0.0089 0.0199;
    -8.8533 224.6563 -0.0360 -0.8097 ];

b = [ -0.0000 0.0000;
    -0.0226 0.0212;
    0.0010 0.0035;
    -0.1601 0.2148 ];

c = [ 1.0000 -0.0050 0.0000 0.0000;
    0.0000 1.0025 0.0000 -0.0000;
    0.1604 0.0084 0.9612 0.2769;
    0.0000 -0.0049 -0.0011 0.0040 ];

d = zeros(4,2);
%% The State space system
system = ss(a,b,c,d);
%% Implementing state feedback
Q = 10 * eye(4);
R = eye(2);
[K,S,e] = lqr(a,b,Q,R)
controlledSystem = ss(a-b*K,b,c,d);
newpoles = eig(controlledSystem);
%figure
%pzplot(controlledSystem);

%% for observer
q = 0.1 * eye(4);
r = 5 * eye(4);
[l,s,e1] = lqr(a',c',q,r);

%% shifting the eigen values
q1 = 15*eye(4);
r1 = 0.1*eye(4);
[l1,s1,e2] = lqr(a',c',q1,r1)