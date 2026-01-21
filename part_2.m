clear; clc; close all;

%% 0. Terms Definition

s = tf('s');
G_i = 5/(s^2+2*s+5);
G_2 = 5/(s*(s+3));
K = 2;
H = 1;
G_plant = G_i * G_2;

sys_ori_cl = feedback(K*G_plant, H);
figure('Name', 'Original Step Response');
grid on;
step(sys_ori_cl);
title('Original Step Response');

%% 1.1 Gain adjustment

K_new = 60;
sys_adj_cl = feedback(K_new*G_plant, H);

figure('Name', 'Gain-adjusted Step Response');
step(sys_adj_cl);
title('Gain-adjusted Step Response');

% A bode plot is drawn first to obtain the ω corresponding to the target pm
figure('Name', 'Frequency Response Analysis for Designing Lag Compensator');
systems = {K*G_plant, K_new*G_plant};
names = {'Original (K=2)', 'Gain Adj (K=60)'};

bode(systems{:});
legend(names);
grid on;

title('Bode Diagram Comparison');
set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1.5);

%% 1.2 Lag compensator 

zc1 = 0.05255; pc1 = 0.0002719;
G_lag1 = (pc1/zc1) * (s + zc1) / (s + pc1);
sys_lag_cl = feedback(K_new*G_plant*G_lag1, H);

figure('Name', 'Lag-compensated Step Response');
step(sys_lag_cl);
title('Lag-compensated Step Response');

%% 1.3 Lead-lag compensator

zc_lead = 0.5307; pc_lead = 0.6426; beta = 0.8256; 
G_lead = (1/beta) * (s + zc_lead) / (s + pc_lead); 
G_leadlag = G_lead * G_lag1; 
sys_leadlag_cl = feedback(K_new*G_plant*G_leadlag, H); 

figure('Name', 'Lead-Lag Step Response');
step(sys_leadlag_cl);
title('Lead-Lag Step Response');

%% 2. Summary Comparison

figure('Name', 'Overall Frequency Response Analysis');
systems = {K*G_plant, K_new*G_plant, K_new*G_plant*G_lag1, K_new*G_plant*G_leadlag};
names = {'Original (K=2)', 'Gain Adj (K=60)', 'Solution 1', 'Lead-Lag'};


bode(systems{:});
legend(names);
grid on;
title('Bode Diagram Comparison');
set(findobj(gcf, 'Type', 'line'), 'LineWidth', 1.5);

n = length(systems);
Gm_db = zeros(n, 1);
Pm_deg = zeros(n, 1);
Wcg_vals = zeros(n, 1);
Wcp_vals = zeros(n, 1);
Stability = cell(n, 1);

% For-loop is used to avoid repetitive work
for i = 1:length(systems)
    [Gm, Pm, Wcg, Wcp] = margin(systems{i});
    
    % Store values in the vectors created above
    Gm_db(i) = 20*log10(Gm);
    Pm_deg(i) = Pm;
    Wcg_vals(i) = Wcg;
    Wcp_vals(i) = Wcp;

    if Pm > 0 && 20*log10(Gm) > 0
        Stability{i} = 'Stable';
    else
        Stability{i} = 'Unstable';
    end
end

ResultsTable = table(names', Gm_db, Pm_deg, Wcg_vals, Wcp_vals, Stability, ...
    'VariableNames', {'System', 'GainMargin_dB', 'PhaseMargin_deg', ...
    'Wcg_Frequency', 'Wcp_Frequency', 'Status'});

disp(ResultsTable);