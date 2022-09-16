%% Method of interpolating noise-free data using a Gaussian with prior precision lambda=200, code largely inspired by gaussInterpDemo.m in Murphy, K.P. Machine Learning: A Probabilistic Perspective; MIT Press, 2012      
% Based on p140 of "Introduction to Bayesian scientific computation" by Calvetti and Somersalo

clear all
close all

%% step1: load experimental data 
SS_expDataArtGP1

%% main algo - part1
%choose
D = 150;


%% interface
Nobs = length(expData.v);
xs = linspace(min([0 min(expData.xobst)]), max([1.0 max(expData.xobst)]), D) ; 

% Noisy observations of the x values at obsNdx
obsNoiseVar = mean(expData.v_stdDev) ; %just take the mean
y =  expData.v;  %=sqrt(obsNoiseVar)*randn(Nobs, 1);
xobs = y;

%%%%% find those indices closest to expData.xobst on the grid defined by xs         
for id=1:Nobs
    obsNdx(id) = round( fzero_curve(1:D,xs,[0 -expData.xobst(id)]) ) ;
end %for id=


%% main algo - part2
hidNdx = setdiff(1:D, obsNdx);


% Make a (D-2) * D tridiagonal matrix
L = spdiags(ones(D-2,1) * [-1 2 -1], [0 1 2], D-2, D);

% prior precicion lambda it only affects the variance,
% not the mean, so we pick a value that results in a pretty plot
lambdas = [10, 200];
filenames = { num2str(lambdas(1)), num2str(lambdas(2))};
names = filenames;

for trial=1:numel(lambdas)
    %names{trial} = sprintf('%4.3f', lambdas(trial));
    lambda = lambdas(trial);
L = L*lambda;


%% Numerically stable method

L1 = L(:, hidNdx);
L2 = L(:, obsNdx);
B11 = L1'*L1;
B12 = L1'*L2;
postDist.Sigma = inv(B11);
postDist.mu = -inv(B11)*B12*xobs;


%% Plot
xbar = zeros(D, 1);
xbar(hidNdx) = postDist.mu;
xbar(obsNdx) = xobs;

sigma = zeros(D, 1);
sigma(hidNdx) = sqrt(diag(postDist.Sigma));
sigma(obsNdx) = 0;

%%%%% plot marginal posterior pm sd as gray band
%choose
factor = 3 ;

%%%conseq: CI is used for plotting purpose
%ini
CI = nan; %by default; confidence interval, e.g. 95%, 99.7% etc.
switch factor
    case 2
        CI = 95;
    case 3
        CI = 99.7;
end
%otherwise keep default value for CI

figure; hold on;
mu = xbar; %overwrite
S2 = sigma.^2;
CI_ys = [mu+factor*sqrt(S2); flipdim(mu-factor*sqrt(S2),1)];
fill([xs'; flipdim(xs',1)], CI_ys, [7 7 7]/8, 'DisplayName',[num2str(CI,"%.1f"),'% marginal\newline{} credibility interval']);

plot(xs(obsNdx), xobs, 'kx', 'markersize', 14, 'linewidth', 3, 'DisplayName','experimental data');
plot(xs, mu, 'k-', 'linewidth', 2, 'DisplayName','interpolation function');

%title(sprintf('%s=%s', '\lambda', names{trial}));

legend('show', 'Location','best');
grid on;

xlabel('x^{obst} [m]');
ylabel('v [m/s]');
axis tight;

axis([0    0.7000   -0.0145    0.3145]); 

box on;

end % next trial
