function [data, grid] = calcReachSet(model)
% Function used to calculate backwards reachable tube for quadcopter
% vertical flight. Outputs grid over which calculations are performed and
% value function at the final time specified by model parameters.

fprintf('Calculating reachable set...\n');

%% Create the grid
grid_min = model.grid_min;                  % Lower corner of computation domain
grid_max = model.grid_max;                  % Upper corner of computation domain
N = [model.grid_count; model.grid_count];   % Number of grid points per dimension
grid = createGrid(grid_min, grid_max, N);   % Create grid

%% Define target set
data0 = shapeRectangleByCorners(grid, model.target(1,:), model.target(2,:));

% Uncomment to visualize target set
% surf(grid.xs{1}, grid.xs{2}, data0);
% contour(grid.xs{1},grid.xs{2},data0,[0 0])

%% Create time vector
tau = 0 : model.dt : model.t_max;

%% Pack problem parameters
% Define dynamic system
dQuad = QuadVerticalFlight(model, 1:2);

% Put grid and dynamic systems into schemeData
schemeData.grid = grid;
schemeData.dynSys = dQuad;
schemeData.accuracy = 'high';
schemeData.uMode = model.uMode;
schemeData.dMode = model.dMode;

%% Compute value function
% Define extra arguments for HJI reachability analysis
HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualizeTarget = true; %plot target set
HJIextraArgs.plotColorT = 'b'; %plot in blue
HJIextraArgs.xTitle = 'Altitude (m)'; %label x-axis
HJIextraArgs.yTitle = 'Vertical Veloctiy (m/s)'; %label y-axis
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.keepLast = true; %keep last value function only 
HJIextraArgs.quiet = true; %suppress "Computing..." messages

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'maxVOverTime', HJIextraArgs);


%% Print completion message
fprintf('Computed reachable set!\n');

% Provide warning if reachable set is empty
if min(data(:)) > 0
    fprintf('\n');
    warning('Safe Set empty! Finalizing test and entering hover mode.');
    fprintf('\n');
end
end
