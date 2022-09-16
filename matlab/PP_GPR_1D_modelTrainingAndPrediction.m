%% Gaussian Process Regression, code largely inspired by demoRegression.m in http://www.gaussianprocess.org/gpml/code/matlab/doc/
% Add to Path all files & folders associated to GPML Matlab Code mentioned above: addpath(genpath('..\B2006Rasmussen\code')) %iot have access to sq_dist.m, covSum.m, etc.        

clear all
close all

%% load experimental data 
SS_expDataArtGP1

%% interface
x  = expData.xobst;
y  = expData.v;
xs = mcv(linspace(0, .7, 100));        

%% algo: code inspired from http://www.gaussianprocess.org/gpml/code/matlab/doc/ 
meanfunc = [];                    % empty: don't use a mean function
covfunc = @covSEiso;              % Squared Exponental covariance function
likfunc = @likGauss;              % Gaussian likelihood

hyp_ini = struct('mean', [], 'cov', [0 0], 'lik', -1);

%%%%% step1: compute/identify hyperparameters by optimizing the (log) marginal likelihood
hyp4 = minimize(hyp_ini, @gp, -100, @infGaussLik, meanfunc, covfunc, likfunc, x, y)

% tune the param lik because I can calculate it experimentally cf expData.v_stdDev  
hyp4.lik = .9*hyp4.lik;

%%%%% step2: make predictions
[ymu ys2 fmu fs2 ] = gp(hyp4, @infGaussLik, meanfunc, covfunc, likfunc, x, y, xs);

%% interface
mu = ymu;
s2 = ys2;

%%%%% step3: plot results
%choose
factor = 3 ;

%%%consequence: CI is used for plotting purpose
%ini
CI = nan; %by default; confidence interval, e.g. 95%, 99.7% etc.
switch factor
    case 2
        CI = 95;
    case 3
        CI = 99.7;
end
%otherwise keep default value for CI

figure;
CI_ys = [mcv(mu+factor*sqrt(s2)); flipdim(mcv(mu-factor*sqrt(s2)),1)];
fill([mcv(xs); flipdim(mcv(xs),1)], CI_ys, [7 7 7]/8, 'EdgeColor', [7 7 7]/8, 'DisplayName', [num2str(CI,"%.1f"),'% confidence interval']); hold on; 

plot(x, y, 'kx','markersize', 14, 'LineWidth',3, 'DisplayName','experimental data'); hold on;
plot(xs, mu, 'k-','markersize', 14, 'LineWidth',2, 'DisplayName','mean function'); hold on;

legend('show', 'Location','best');
grid on;

xlabel('x^{obst} [m]');
ylabel('v [m/s]');
%ylim([0 .3]); 

axis tight;
%set(gca,'xtick',[]) %remove numbers shown on axis
%set(gca,'ytick',[]) %remove numbers shown on axis

box on;
