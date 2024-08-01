clear all;
clc;
km=0.022; 
ke=0.4;
R=1.7;
r=0.0325;
Mp=1;
Mw=0.03;
Ip=0.0012;
Iw=0.000016;
l=0.05;
g=9.81;
teta_init=0;
teta_dot_init=0;
x_init=0;
x_dot_init=0;
beta = 2*Mw+2*Iw/r^2+Mp;
alpha = Ip*beta+2*Mp*l^2*(Mw+Iw/r^2);
%------------------------------------------------%
A = [0 1 0 0;
    0 (2*km*ke*(Mp*l*r-Ip-Mp*l^2))/(R*r^2*alpha) (Mp^2*g*l^2)/alpha 0;
    0 0 0 1;
    0 (2*km*ke*(r*beta-Mp*l))/(R*r^2*alpha) (Mp*g*l*beta)/alpha 0]
B = [0;
    (2*km*(Ip+Mp*l^2-Mp*l*r))/(R*r*alpha);
    0;
    (2*km*(Mp*l-r*beta))/R*r*alpha]
C = [1 0 0 0;
     0 0 1 0]       % ma tran C dung de khao sat tinh quan sat duoc cua he thong
D = [0; 
    0]              % ma tran D = 0 de ngo vao khong lam anh huong den ngo ra cua he thong
%------------------------------------------------%
%---Khao sat tinh dieu khien duoc cua he thong---%
P = [B A*B A^2*B A^3*B];
rank(P);
%---Khao sat tinh quan sat duoc cua he thong---%
L = [C; C*A; C*A^2; C*A^3];
rank(L);
%------------------------------------------------%
%---TINH HAM TRUYEN CUA HE THONG---%

[num,den] = ss2tf(A,B,C,D)
HTXE1 = tf(num(1,:),den)
HTXE2 = tf(num(2,:),den)

%rlocus (A,B,C,D)
pzmap (A,B,C,D)

nghiem = roots(den)