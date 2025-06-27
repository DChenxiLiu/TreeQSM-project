%% Run TreeQSM with LAS File using Lidar Toolbox
% Simple script to load a LAS file and run TreeQSM
% Requires MATLAB Lidar Toolbox

clear all; close all; clc;

%% Configuration
% CHANGE THIS to your LAS file path
las_file = '/Users/doraliu/Library/CloudStorage/OneDrive-UniversityofWaterloo/hullet/individualtree_hullet/208_off_individual_tree/208_GS0011.las';

% TreeQSM parameters
downsample_threshold = 300000;  % Downsample if more than this many points
filter_point_cloud = true;      % Apply filtering to remove noise
auto_parameters = true;         % Use automatic parameter selection

%% Step 0: List all LAS files and split into train/test sets
las_folder = fileparts(las_file); % Use the same folder as 208_GS0007.las
las_files_struct = dir(fullfile(las_folder, '*.las'));
las_files = {las_files_struct.name};
num_files = numel(las_files);

% Randomly permute and split
rng('shuffle'); % For true randomness
perm = randperm(num_files);
train_idx = perm(1:round(0.7*num_files));
test_idx = perm(round(0.7*num_files)+1:end);
train_files = las_files(train_idx);
test_files = las_files(test_idx);

fprintf('Total LAS files: %d\n', num_files);
fprintf('Training set: %d files\n', numel(train_files));
fprintf('Testing set: %d files\n', numel(test_files));

% Example: loop over training files (replace this with your main loop later)
% for i = 1:numel(train_files)
%     this_file = fullfile(las_folder, train_files{i});
%     % ... load and process this_file ...
% end

%% Step 1: Setup
addpath(genpath('src'));
disp('TreeQSM paths added');

%% Step 2: Load LAS file
disp(['Loading LAS file: ', las_file]);

try
    % Create LAS file reader
    lasReader = lasFileReader(las_file);
    
    % Read point cloud
    ptCloud = readPointCloud(lasReader);
    P = double(ptCloud.Location);
    
    % Debug: Check initial dimensions
    fprintf('Initial point cloud size: %d x %d\n', size(P));
    if size(P, 2) ~= 3
        error('Point cloud should have 3 columns (X,Y,Z), but has %d columns', size(P, 2));
    end
    
    % Remove invalid points
    valid = all(isfinite(P), 2);
    P = P(valid, :);
    
    % Debug: Check dimensions after filtering invalid points
    fprintf('After removing invalid points: %d x %d\n', size(P));
    
    disp(['Loaded ', num2str(size(P,1)), ' valid points']);
    
    % Display point cloud info
    fprintf('\nPoint Cloud Statistics:\n');
    fprintf('X range: %.2f to %.2f m\n', min(P(:,1)), max(P(:,1)));
    fprintf('Y range: %.2f to %.2f m\n', min(P(:,2)), max(P(:,2)));
    fprintf('Z range: %.2f to %.2f m\n', min(P(:,3)), max(P(:,3)));
    
catch ME
    error(['Failed to load LAS file: ', ME.message]);
end

%% Step 3: Preprocessing
% Center the point cloud
P_mean = mean(P);
P = P - P_mean;
disp('Point cloud centered');

% Debug: Check dimensions after centering
fprintf('After centering: %d x %d\n', size(P));

% Downsample if needed
if size(P,1) > downsample_threshold
    fprintf('Downsampling from %d to %d points...\n', size(P,1), downsample_threshold);
    idx = randperm(size(P,1), downsample_threshold);
    P = P(idx, :);
    
    % Debug: Check dimensions after downsampling
    fprintf('After downsampling: %d x %d\n', size(P));
end

%% Step 4: Filter point cloud (optional)
if filter_point_cloud
    disp('Filtering point cloud...');
    
    % Define filter parameters
    filter_inputs.filter.k = 15;            % k-nearest neighbors
    filter_inputs.filter.nsigma = 2;        % Standard deviation threshold
    filter_inputs.filter.radius = 0;        % Radius-based (0 = disabled)
    filter_inputs.filter.ncomp = 5;         % Minimum component size
    filter_inputs.filter.PatchDiam1 = 0.05;
    filter_inputs.filter.BallRad1 = 0.075;
    filter_inputs.filter.EdgeLength = 0;    % Voxel downsampling (0 = disabled)
    filter_inputs.filter.plot = 0;
    
    P_original = P;
    
    % Debug: Check what filtering returns
    filter_result = filtering(P, filter_inputs);
    fprintf('Filtering returned: %d x %d\n', size(filter_result));
    
    % Check if filtering returned the expected format
    if size(filter_result, 2) == 3
        P = filter_result;
        fprintf('Filtering removed %d points (%.1f%%)\n', ...
            size(P_original,1) - size(P,1), ...
            (size(P_original,1) - size(P,1))/size(P_original,1)*100);
    else
        % If filtering didn't return expected format, skip filtering
        warning('Filtering function returned unexpected format. Skipping filtering step.');
        fprintf('Expected 3 columns, got %d columns\n', size(filter_result, 2));
        P = P_original;
    end
    
    % Debug: Check dimensions after filtering
    fprintf('After filtering: %d x %d\n', size(P));
end

%% Step 5: Visualize point cloud
% Validate point cloud dimensions before plotting
if size(P, 2) ~= 3
    error('Point cloud must have 3 columns (X,Y,Z) for plotting, but has %d columns', size(P, 2));
end

if size(P, 1) == 0
    error('Point cloud is empty after processing');
end

figure('Name', 'LAS Point Cloud', 'Position', [100, 100, 800, 600]);
plot3(P(:,1), P(:,2), P(:,3), '.', 'MarkerSize', 1, 'Color', [0.2 0.6 0.2]);
title(sprintf('%s\n%d points', las_file, size(P,1)));
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal; view(3); grid on;
rotate3d on;

%% Step 6: Setup TreeQSM parameters
if auto_parameters
    % Automatic parameter selection based on tree size
    disp('Using automatic parameter selection...');
    inputs = define_input(P, 2, 3, 2);  % 2 values for each parameter
    
    % Validate automatic parameters
    if isnan(inputs.PatchDiam1(1)) || isnan(inputs.PatchDiam2Min(1)) || isnan(inputs.PatchDiam2Max(1))
        warning('Automatic parameter selection failed (returned NaN). Using manual parameters instead.');
        auto_parameters = false;  % Switch to manual mode
    else
        fprintf('Automatic parameters:\n');
        fprintf('  PatchDiam1: %.3f, %.3f\n', inputs.PatchDiam1);
        fprintf('  PatchDiam2Min: %.3f, %.3f\n', inputs.PatchDiam2Min);
        fprintf('  PatchDiam2Max: %.3f, %.3f\n', inputs.PatchDiam2Max);
    end
end

if ~auto_parameters
    % Manual parameters - suitable for medium-large trees
    disp('Using manual parameters...');
    
    % Calculate reasonable parameters based on point cloud size
    point_density = size(P,1) / (range(P(:,1)) * range(P(:,2)) * range(P(:,3)));
    
    if point_density > 50000  % High density
        inputs.PatchDiam1 = [0.05 0.10];
        inputs.PatchDiam2Min = [0.02 0.04 0.06];
        inputs.PatchDiam2Max = [0.05 0.12];
    else  % Medium density
        inputs.PatchDiam1 = [0.08 0.12];
        inputs.PatchDiam2Min = [0.04 0.05 0.06];
        inputs.PatchDiam2Max = [0.07 0.10];
    end
    
    inputs.BallRad1 = inputs.PatchDiam1 + 0.015;
    inputs.BallRad2 = inputs.PatchDiam2Max + 0.01;
    
    fprintf('Manual parameters:\n');
    fprintf('  PatchDiam1: %.3f, %.3f\n', inputs.PatchDiam1);
    fprintf('  PatchDiam2Min: %.3f, %.3f\n', inputs.PatchDiam2Min);
    fprintf('  PatchDiam2Max: %.3f, %.3f\n', inputs.PatchDiam2Max);
end

% Set other parameters
inputs.OnlyTree = 1;
inputs.Tria = 0;
inputs.Dist = 1;
inputs.MinCylRad = 0.0025;
inputs.ParentCor = 1;
inputs.TaperCor = 1;
inputs.GrowthVolCor = 0;

% Output settings
[~, filename, ~] = fileparts(las_file);
inputs.name = filename;
inputs.tree = 1;
inputs.model = 1;
inputs.savemat = 1;
inputs.savetxt = 0;
inputs.plot = 0; % Disable QSM visualization
inputs.disp = 1;

% Get the directory of this script
script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');

% Create results folder (ensure it exists before running treeqsm)
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Step 1: Preprocess all training point clouds first
preprocessed_P = cell(numel(train_files), 1);
preprocessed_valid = false(numel(train_files), 1);

for i = 1:numel(train_files)
    this_file = fullfile(las_folder, train_files{i});
    try
        lasReader = lasFileReader(this_file);
        ptCloud = readPointCloud(lasReader);
        P = double(ptCloud.Location);
        valid = all(isfinite(P), 2);
        P = P(valid, :);
        if size(P,2) ~= 3 || isempty(P)
            warning('Invalid point cloud in %s, skipping.', train_files{i});
            continue;
        end
        % Center
        P_mean = mean(P);
        P = P - P_mean;
        % Downsample if needed
        if size(P,1) > downsample_threshold
            idx = randperm(size(P,1), downsample_threshold);
            P = P(idx, :);
        end
        % Filter (optional)
        if filter_point_cloud
            filter_inputs.filter.k = 15;
            filter_inputs.filter.nsigma = 2;
            filter_inputs.filter.radius = 0;
            filter_inputs.filter.ncomp = 5;
            filter_inputs.filter.PatchDiam1 = 0.05;
            filter_inputs.filter.BallRad1 = 0.075;
            filter_inputs.filter.EdgeLength = 0;
            filter_inputs.filter.plot = 0;
            filter_result = filtering(P, filter_inputs);
            if size(filter_result,2) == 3
                P = filter_result;
            end
        end
        preprocessed_P{i} = P;
        preprocessed_valid(i) = true;
    catch ME
        warning('Failed to preprocess %s: %s', train_files{i}, ME.message);
        preprocessed_P{i} = [];
    end
end

%% Step 2: Monte Carlo parameter search on training set
num_trials = 20; % Number of random parameter sets to try per tree
results_table = [];

for i = 1:numel(train_files)
    if ~preprocessed_valid(i)
        continue;
    end
    P = preprocessed_P{i};
    [~, filename, ~] = fileparts(train_files{i});
    fprintf('\n--- Training Tree %d/%d: %s ---\n', i, numel(train_files), train_files{i});
    for trial = 1:num_trials
        % Randomly sample parameters within reasonable ranges
        PatchDiam1 = rand(1,2)*0.1 + 0.03; % [0.03, 0.13]
        PatchDiam2Min = rand(1,2)*0.04 + 0.01; % [0.01, 0.05]
        PatchDiam2Max = rand(1,2)*0.06 + 0.03; % [0.03, 0.09]
        BallRad1 = PatchDiam1 + 0.015;
        BallRad2 = PatchDiam2Max + 0.01;
        % Set up inputs
        inputs.PatchDiam1 = PatchDiam1;
        inputs.PatchDiam2Min = PatchDiam2Min;
        inputs.PatchDiam2Max = PatchDiam2Max;
        inputs.BallRad1 = BallRad1;
        inputs.BallRad2 = BallRad2;
        inputs.OnlyTree = 1;
        inputs.Tria = 0;
        inputs.Dist = 1;
        inputs.MinCylRad = 0.0025;
        inputs.ParentCor = 1;
        inputs.TaperCor = 1;
        inputs.GrowthVolCor = 0;
        inputs.name = filename;
        inputs.tree = 1;
        inputs.model = trial;
        inputs.savemat = 0;
        inputs.savetxt = 0;
        inputs.plot = 0;
        inputs.disp = 0;
        % Run TreeQSM
        try
            QSM = treeqsm(P, inputs);
            if ~isempty(QSM)
                dbh = QSM(1).treedata.DBHqsm * 100; % cm
            else
                dbh = NaN;
            end
        catch
            dbh = NaN;
        end
        % Store results
        results_table = [results_table; {filename, trial, PatchDiam1, PatchDiam2Min, PatchDiam2Max, dbh}];
    end
end

results_table = cell2table(results_table, 'VariableNames', {'TreeID','Trial','PatchDiam1','PatchDiam2Min','PatchDiam2Max','DBHcm'});

%% Step 7: Run TreeQSM
fprintf('\n========== Running TreeQSM ==========\n');

tic;

% Run QSM reconstruction
QSM = treeqsm(P, inputs);

elapsed = toc;
fprintf('QSM reconstruction completed in %.1f seconds\n', elapsed);

%% Step 8: Save and Display results
if ~isempty(QSM)
    for k = 1:numel(QSM)
        % Manual save to results directory for each model
        save_filename = sprintf('QSM_%s_t1_m%d.mat', filename, k);
        save_path = fullfile(results_dir, save_filename);
        save(save_path, 'QSM');
        
        fprintf('\n========== Tree Measurements (Model %d) ==========' , k);
        fprintf('\nTree Height:        %.2f m\n', QSM(k).treedata.TreeHeight);
        fprintf('Trunk Volume:       %.1f L\n', QSM(k).treedata.TrunkVolume * 1000);
        fprintf('Branch Volume:      %.1f L\n', QSM(k).treedata.BranchVolume * 1000);
        fprintf('Total Volume:       %.1f L\n', QSM(k).treedata.TotalVolume * 1000);
        fprintf('DBH:                %.1f cm\n', QSM(k).treedata.DBHqsm * 100);
        fprintf('Number of Branches: %d\n', QSM(k).treedata.NumberBranches);
        fprintf('Max Branch Order:   %d\n', QSM(k).treedata.MaxBranchOrder);
        
        if isfield(QSM(k), 'pmdistance')
            fprintf('\n========== Model Quality ==========' );
            fprintf('\nMean point-model distance: %.1f mm\n', QSM(k).pmdistance.mean * 1000);
            fprintf('Std deviation:             %.1f mm\n', QSM(k).pmdistance.std * 1000);
        end
        
        fprintf('\n========== Output Files ==========' );
        fprintf('\nResults saved in: results/QSM_%s_t1_m%d.mat\n', filename, k);
    end
else
    warning('QSM reconstruction failed. Check your point cloud data.');
end

disp(' ');
disp('Script completed successfully!');
