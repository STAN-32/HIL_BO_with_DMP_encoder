clear;close all;clc;
load('Mul_dof.mat');

%% param set for simulink
% AO param
params = struct();
K = 10;
yita = 4;
high_freq = 5*2*pi;
low_freq = pi/5;
M = 5;
w0 = [low_freq:(high_freq-low_freq)/(M-1):high_freq];
w0 = [pi;2*pi;8.17;10*pi;44];
A = 1;
wt = pi;

% sim param
during = 50;

% DMP param
alphaz = 30;
N = 20;
g = 0;

% pack up into structure
params.K = K;
params.yita = yita;
params.high_freq = high_freq;
params.low_freq = low_freq;
params.M = M;
params.w0 = w0;
params.A = A;
params.wt = wt;

% sim param
params.during = during;

% DMP param
params.alphaz = alphaz;
params.N = N;
params.g = g;

%%
Traj_demo = hip;
weight_hip = get_weights(Traj_demo,params);





