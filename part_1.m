clear; clc; close all;

%% 0. Terms Definition

s = tf('s');
G_v = 0.02 / (4*s + 1);
G_p = 70 / (50*s + 1);
H = 1 / (12*s + 1);
G_plant = G_v * G_p;

sys_ori_cl = feedback(G_plant, H);
figure('Name', 'Section 1.1: Original System');
subplot(2,1,1);
rlocus(G_plant * H); hold on;
title('Root Locus - Original System');

subplot(2,1,2);
step(sys_ori_cl);
title('Original Step Response');
grid on;

%% 1.1 Gain Adjustment

zeta = 0.7;
K_adj = 1.1222;

sys_adj_open = K_adj * G_plant * H;
sys_adj_cl = feedback(K_adj * G_plant, H);

figure('Name', 'Section 1.2: Gain Adjustment');
subplot(2,1,1);
rlocus(G_plant * H); hold on;
title('Root Locus - Gain Adjustment');
sgrid(zeta, 0);

subplot(2,1,2);
step(sys_adj_cl);
title(['Step Response (K = ', num2str(K_adj), ')']);
grid on;

%% 1.2 1st PD

z_pd1 = 0.1791;
KD1 = 11.414;

Gc_pd1 = KD1 * (s + z_pd1);
sys_pd1_open = Gc_pd1 * G_plant * H;
sys_pd1_cl = feedback(Gc_pd1 * G_plant, H);

figure('Name', 'Section 1.3: 1st PD Controller');
subplot(2,1,1);
rlocus(sys_pd1_open / KD1); % Plot RL of plant + zero
title('Root Locus - 1st PD Controller');
sgrid(zeta, 0);

subplot(2,1,2);
step(sys_pd1_cl);
title('Step Response - 1st PD Controller');
grid on;

%% 1.3 2nd PD

zc2 = 0.057;
pc2 = 0.112;
KD2 = 2.1030;

Gc_pd2 = KD2 * (s + zc2) / (s + pc2);
sys_pd2_open = Gc_pd2 * G_plant * H;
sys_pd2_cl = feedback(Gc_pd2 * G_plant, H);

figure('Name', 'Section 1.4: 2nd PD Controller (Ogata)');
subplot(2,1,1);
rlocus((s + zc2) / (s + pc2) * G_plant * H);
title('Root Locus - 2nd PD Controller');
sgrid(zeta, 0);

subplot(2,1,2);
step(sys_pd2_cl);
title('Step Response - 2nd PD Controller');
grid on;

%% 1.4 PID 

% GC(s) = KC * GI(s) * GD(s)
KC = 2.1030;
GI = (s + 0.02222) / (s + 0.00176);
GD = (s + 0.057) / (s + 0.112);

Gc_pid = KC * GI * GD;
sys_pid_open = Gc_pid * G_plant * H;
sys_pid_cl = feedback(Gc_pid * G_plant, H);

figure('Name', 'Section 1.5: PID Controller');
subplot(2,1,1);
rlocus(sys_pid_open / KC);
title('Root Locus - PID Controller');
sgrid(zeta, 0);

subplot(2,1,2);
step(sys_pid_cl);
title('Step Response - PID Controller');
grid on;

%% 2. Summary Comparison

figure('Name', 'System Comparison');
systems = {sys_adj_cl, sys_pd1_cl, sys_pd2_cl, sys_pid_cl}; 
names = {'Original P', '1st PD', '2nd PD', 'PID'};

step(systems{:}); 
legend(names);
grid on;

n = length(systems);
zetas = zeros(n, 1);
overshoots = zeros(n, 1);
settlingTimes = zeros(n, 1);
ess = zeros(n, 1);

% For-loop is used to avoid repetitive work
for i = 1:n
    S = systems{i};
    info = stepinfo(S);
    
    zetas(i) = min(damp(S));         
    overshoots(i) = info.Overshoot;   
    settlingTimes(i) = info.SettlingTime;
    ess(i) = abs(1 - dcgain(S));   
end

ResultsTable = table(names', zetas, overshoots, settlingTimes, ess, ...
    'VariableNames', {'System', 'Damping_Ratio', 'Overshoot_Pct', ...
    'SettlingTime_s', 'Ess'});

disp(ResultsTable);