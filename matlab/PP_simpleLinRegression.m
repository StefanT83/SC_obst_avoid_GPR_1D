%% Simple Linear Regression, code largely inspired by regression_line_ci.m in Gutman, B. Linear regression confidence interval. MATLAB Central File Exchange. https://www.mathworks.com/matlabcentral/fileexchange/39339-linear-regression-confidence-interval, Accessed on June 14, 2022.
% Add to Path all files & folders associated to library mentioned above, e.g. addpath(genpath('..\matl_tbox'))            

%% step1: load experimental data 
SS_expDataArtGP1

%% interface
x1=expData.xobst ;
x2=expData.v; 

%% step2: apply Simple Linear Regression
%choose
x1_range = [0 .7];

%% main 
X2  = []; %ini
XX1 = []; %ini

for idx=1:length(x1)
    X2  = [X2; ...
           x2(idx)];
    XX1 = [XX1; ...
           x1(idx) 1];
end

theta = (transpose(XX1)*XX1)\(transpose(XX1)*X2)
a = theta(1); %slope 
b = theta(2); %intercept

%%% compute confidence band
%choose
alpha = 1-.997; %this will generate a 100*(1-alpha) [%] confidence interval for the true mean mu: see [B1977Brown,p272(up)]

[confid_bnd_up, confid_bnd_low, x1_axis] = regression_line_ci(alpha,[b a],x1,x2,1e2,min(x1_range),max(x1_range)) ; %note: requires matlab toolbox "Statistics and Machine learning toolbox"

%%% plotting
figure;

%plot confidence band
CI_ys = [mcv(confid_bnd_up); flipdim(mcv(confid_bnd_low),1)];
fill([mcv(x1_axis); flipdim(mcv(x1_axis),1)], CI_ys, [7 7 7]/8, 'DisplayName', [num2str(100*(1-alpha),"%.1f"),'% confidence band']); hold on; 

%plot data
plot(x1(1:end),x2(1:end),'kx','markersize', 14, 'LineWidth',3, 'DisplayName','experimental data'); hold on;  

%plot regression line
plot(x1_range, a*x1_range+b,'k-','markersize', 14, 'linewidth',2, 'DisplayName','estimated mean'); 

grid on; box on;

xlabel('x^{obst} [m]');
ylabel('v [m/s]');

axis([0 .7 -0.0143 0.3220]); %same as for GP figure


legend('show', 'location','best');

