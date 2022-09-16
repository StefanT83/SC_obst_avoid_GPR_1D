%% Bayesian Neural Network, code largely inspired by: (i) mlpRegEvidenceDemo.m in Murphy, K.P. Machine Learning: A Probabilistic Perspective; MIT Press, 2012. (ii) demev1.m in Nabney, I. Netlab. MATLAB Central File Exchange. https://www.mathworks.com/matlabcentral/fileexchange/2654-netlab, Retrieved April 21, 2022
% Add to Path all files & folders associated to library mentioned above, e.g. addpath(genpath('..\B2012Murphy\code'))  and  addpath(genpath('..\B2002Nabney\code'))                   

%% step1: load experimental data 
SS_expDataArtGP1

%% interface
x = mcv(expData.xobst);
t = mcv(expData.v); 


%% Plot the data 
%h = figure;
nplot = 200;
plotvals = linspace(0, 30, nplot)';

% Set up network parameters.
nin = 1;		% Number of inputs.
nhidden = 3;		% Number of hidden units.
nout = 1;		% Number of outputs.
alpha = 0.01;		% Initial prior hyperparameter.
beta_init = 50.0;	% Initial noise hyperparameter.

% Create and initialize network weight vector.
net = mlp(nin, nhidden, nout, 'linear', alpha, beta_init);

% Set up vector of options for the optimiser.
nouter = 3;			% Number of outer loops.
ninner = 1;			% Number of innter loops.
options = zeros(1,18);		% Default options vector.
options(1) = 1;			% This provides display of error values.
options(2) = 1.0e-7;		% Absolute precision for weights.
options(3) = 1.0e-7;		% Precision for objective function.
options(14) = 500;		% Number of training cycles in inner loop.

% Train using scaled conjugate gradients, re-estimating alpha and beta.
for k = 1:nouter
    net = netopt(net, options, x, t, 'scg');
    
    [net, gamma] = evidence(net, x, t, ninner);
    fprintf(1, '\nRe-estimation cycle %d:\n', k);
    fprintf(1, '  alpha =  %8.5f\n', net.alpha);
    fprintf(1, '  beta  =  %8.5f\n', net.beta);
    fprintf(1, '  gamma =  %8.5f\n\n', gamma);
    disp(' ')
end

% stefan: ad-hoc/not the proper way of doing things: modify the param beta because I can calc it experimentally cf expData.v_stdDev: modify the result of the optimiz: Note this is not the proper way of doing it, instead this param should be clipped beforehand, and kept outside the optimiz fct above, and leave the optimiz running on the other 2 param only 
%net.beta = 50000;


% Evaluate error bars.
[y, sig2] = netevfwd(mlppak(net), net, x, t, plotvals);
sig = sqrt(sig2);

% Plot the data, the original function, and the trained network function.
[y, z] = mlpfwd(net, plotvals);
%figure(h); hold on;
%plot(plotvals, y, '-r', 'linewidth', 3);
%xlabel('Input')
%ylabel('Target')
%plot(plotvals, y + sig, ':b');
%plot(plotvals, y - sig, ':b');
%legend('data', 'function', 'network', 'error bars');

%%% plotting
figure;

%plot confidence band
CI_ys = [mcv(y + 3*sig); flipdim(mcv(y - 3*sig),1)];
fill([mcv(plotvals); flipdim(mcv(plotvals),1)], CI_ys, [7 7 7]/8, 'DisplayName', [num2str(99.7,"%.1f"),'% error bars']); hold on; 

%plot data
plot(x,t,'kx','markersize', 14, 'LineWidth',3, 'DisplayName','experimental data'); hold on;  

%plot 'posterior mean prediction'
plot(plotvals, y,'k-','markersize', 14, 'linewidth',2, 'DisplayName','posterior mean prediction'); 


grid on; box on;

xlabel('x^{obst} [m]');
ylabel('v [m/s]');

axis([0 .7 -0.0143 0.3220]); %same as for GP figure

legend('show', 'location','best');

%{
disp(' ')
disp('Notice how the confidence interval spanned by the ''error bars'' is')
disp('smaller in the region of input space where the data density is high,')
disp('and becomes larger in regions away from the data.')
disp(' ')
%}

%printPmtkFigure('demoEvidenceReg')



