function weights = get_weights(Traj,params)

%% Some illustration
% This function is used to generate the DMP weights of the given Traj.
% For the sake of origin test, This function is not fully programed for its
% propose.
% It's Okay to change the content to plot gragh and save it.

%% unpack params
% % % % % % % % % AO param
% % % % % % % % K = 10;
% % % % % % % % yita = 4;
% % % % % % % % high_freq = 5*2*pi;
% % % % % % % % low_freq = pi/5;
% % % % % % % % M = 5;
% % % % % % % % w0 = [low_freq:(high_freq-low_freq)/(M-1):high_freq];
% % % % % % % % w0 = [pi;2*pi;8.17;10*pi;44];
% % % % % % % % A = 1;
% % % % % % % % wt = pi;
% % % % % % % % 
% % % % % % % % % sim param
% % % % % % % % during = 50;
% % % % % % % % 
% % % % % % % % % DMP param
% % % % % % % % alphaz = 30;
% % % % % % % % 
% % % % % % % % g = 0;
K = params.K;
yita = params.yita;
high_freq = params.high_freq;
low_freq = params.low_freq;
M = params.M;
w0 = params.w0;
A = params.A;
wt = params.wt;
during = params.during;
alphaz = params.alphaz;
N = params.N;
g = params.g;

% Demo set
Traj_demo = Traj; % change the demo

sim('DMP_model.slx');


%% manual DMP reproduce
Time = weight_batch.time;
weight_data = weight_batch.data(:,2:N+1);
final_weight = weight_data(end,:);

% plot DMP reasult. params are copied from S-fun
c = [0:2*pi/(N-1):2*pi];
h = 2.5*N;
r = 1;
g = 0;
freq = 1;
omiga = 2*pi*freq;
betaz = alphaz/4;

vector_w = final_weight;% back to 0-1


t_s = 0.001;
t_DMP = [0:t_s:1]*omiga;
y = [];
z = [];
for i = 1:length(t_DMP)
    phi = t_DMP(i);
    Phi_i = exp(h*(cos(phi-c)-1));
    f_nolinear = sum(vector_w.*Phi_i)*r/sum(Phi_i);
    if i == 1
        z(i) = 0;
        y(i) = 0;
    else
        z(i) = z(i-1) + omiga*(alphaz*(betaz*(g-y(i-1))-z(i-1))+f_nolinear)*t_s;
        y(i) = y(i-1) + omiga*z(i-1)*t_s;
    end
end

y_demo = Traj_demo;%*pi/180; % change the demo
y = y+y_demo(1);% weight直接确定了权重，初值直接确定了bias
plot(100*t_DMP/(omiga),y,'r--','linewidth',12);hold on;plot(100*t_DMP/(omiga),y_demo,'linewidth',2);
% title('DMP Result MSE = ',num2str(mse(y - y_demo)))
% title('DMP Output')
set(gcf, 'Position', [0 0 2084 795])
set(gca,'FontSize',35)%18
set(gca,'looseInset',[0 0 0 0])
xlabel('Gait cycle(%)', 'FontSize', 30);%20
ylabel('Angle(rad)', 'FontSize', 30);
% axis off;
legend('DMP','Origin','Interpreter', 'LaTex','FontSize', 20,'location', 'best');

fig = gca;
print('-r300', '-dpng','compare.png');% 第一个参数为图像句柄


%% end process
% clearvars -except stu1 stu2 weight_batch y_demo t_DMP gm
weights = final_weight;
close all;
end


