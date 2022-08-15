%% clear variables
clc, clear all, close all;

%% Load Variable
load("errores_mpc.mat")
load("errores_mpc_neuronal.mat")

ts = 0.01;
%% Calculo IAE
q1_e_mpc =  trapz(ts,E_mpc(1:end).^2);
q2_e_mpc=  trapz(ts,E_mpc(2:end).^2);
q1p_e_mpc =  trapz(ts,E_mpc(3:end).^2);
q2p_e_mpc =  trapz(ts,E_mpc(4:end).^2);

q1_e_mpc_n =  trapz(ts,E_mpc_neuronal(1:end).^2);
q2_e_mpc_n=  trapz(ts,E_mpc_neuronal(2:end).^2);
q1p_e_mpc_n =  trapz(ts,E_mpc_neuronal(3:end).^2);
q2p_e_mpc_n =  trapz(ts,E_mpc_neuronal(4:end).^2);

ERROR = [q1_e_mpc, q1_e_mpc_n;...
         q2_e_mpc, q2_e_mpc_n;...
         q1p_e_mpc, q1p_e_mpc_n;...
         q2p_e_mpc, q2p_e_mpc_n];

% color propreties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 9; %11
fontsizeLegend = 9;
fontsizeTicks = 9;
fontsizeTitel = 9;
sizeX = 900; % size figure
sizeY = 300; % size figure

C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;

figure()
% set(gcf, 'PaperPositionMode', 'manual');
bar(ERROR);
grid on;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel({'$\omega_{ref}$','$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLabel)
ylabel('$\textrm{ISE}$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\textrm{MPC}$','$\textrm{MPC Adaptativo}$'},'interpreter','latex','fontsize',fontsizeLegend)
