%% Run TreeQSM with LAS File using Lidar Toolbox
% Simple script to load a LAS file and run TreeQSM
% Requires MATLAB Lidar Toolbox

clear all; close all; clc;

%% Configuration
% CHANGE THIS to your LAS file path 
% For Windows users: 'C:\Users\c72liu\OneDrive - University of Waterloo\hullet\individualtree_hullet\208_off_individual_tree\208_GS0011.las'
% For macOS users: '/Users/c72liu/Library/CloudStorage/OneDrive-UniversityofWaterloo/hullet/individualtree_hullet/208_off_individual_tree/208_GS0011.las'

las_file = 'C:\Users\c72liu\OneDrive - University of Waterloo\hullet\individualtree_hullet\208_off_individual_tree\208_GS0011.las';

% TreeQSM parameters
downsample_threshold = 300000;  % Downsample if more than this many points
filter_point_cloud = true;      % Apply filtering to remove noise
auto_parameters = true;         % Use automatic parameter selection

%% Step 0: Data Preparation – List and split LAS files
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

%% Step 1: Environment Setup – Add TreeQSM paths
addpath(genpath('src'));
disp('TreeQSM paths added');

%% Step 2: Single Tree Visualization & Parameter Setup
% This section is for visualization and parameter setup demonstration only.
% It does NOT affect the batch Monte Carlo search below.

% Set to true to visualize and inspect a single tree, false to skip this step
visualize_single_tree = true; 

if visualize_single_tree
    disp('--- Single Tree Visualization and Parameter Setup ---');
    
    % Select the LAS file to visualize
    single_file = las_file;
    
    % Read the LAS file using MATLAB's Lidar Toolbox
    lasReader = lasFileReader(single_file);
    ptCloud = readPointCloud(lasReader);
    P = double(ptCloud.Location); % Extract XYZ coordinates
    
    % Remove any invalid points (NaN or Inf)
    valid = all(isfinite(P), 2);
    P = P(valid, :);
    
    % Center the point cloud at the origin
    P_mean = mean(P);
    P = P - P_mean;
    
    % Downsample the point cloud if it exceeds the threshold
    if size(P,1) > downsample_threshold
        idx = randperm(size(P,1), downsample_threshold);
        P = P(idx, :);
    end
    
    % Optionally filter the point cloud to remove noise
    if filter_point_cloud
        % Filtering is now optional and can be skipped by setting filter_point_cloud = false
        % To skip filtering, set filter_point_cloud = false in the configuration section above.
        % If you want to force skip filtering here, comment out or remove the block below.
        % Example filter parameters (uncomment and adjust as needed):
        % filter_inputs.filter.k = 25;
        % filter_inputs.filter.nsigma = 3;
        % filter_inputs.filter.radius = 0;
        % filter_inputs.filter.ncomp = 2;
        % filter_inputs.filter.PatchDiam1 = 0.05;
        % filter_inputs.filter.BallRad1 = 0.075;
        % filter_inputs.filter.EdgeLength = 0;
        % filter_inputs.filter.plot = 1;
        % filter_result = filtering(P, filter_inputs);
        % if size(filter_result,2) == 3
        %     P = filter_result;
        % end
    end
    
    % Visualize the processed point cloud
    figure('Name', 'LAS Point Cloud', 'Position', [100, 100, 800, 600]);
    plot3(P(:,1), P(:,2), P(:,3), '.', 'MarkerSize', 1, 'Color', [0.2 0.6 0.2]);
    title(sprintf('%s\n%d points', single_file, size(P,1)));
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal; view(3); grid on; rotate3d on;
    drawnow; % Force MATLAB to update the figure window
    
    % Parameter setup: automatic or manual
    % -------------------------------------------------------------
    % Parameter selection: automatic or manual
    % This block sets QSM parameters either automatically (using define_input)
    % or manually based on point density if automatic selection fails or is disabled.
    % -------------------------------------------------------------
    if auto_parameters
        disp('Using automatic parameter selection...');
        inputs = define_input(P, 2, 3, 2);
        % If automatic selection fails, fall back to manual
        if isnan(inputs.PatchDiam1(1)) || isnan(inputs.PatchDiam2Min(1)) || isnan(inputs.PatchDiam2Max(1))
            warning('Automatic parameter selection failed (returned NaN). Using manual parameters instead.');
            auto_parameters = false;
        else
            fprintf('Automatic parameters:\n');
            fprintf('  PatchDiam1: %.3f, %.3f\n', inputs.PatchDiam1);
            fprintf('  PatchDiam2Min: %.3f, %.3f\n', inputs.PatchDiam2Min);
            fprintf('  PatchDiam2Max: %.3f, %.3f\n', inputs.PatchDiam2Max);
        end
    end
    
    % Manual parameter setup if auto failed or not selected
    if ~auto_parameters
        disp('Using manual parameters...');
        % Estimate point density for parameter selection
        point_density = size(P,1) / (range(P(:,1)) * range(P(:,2)) * range(P(:,3)));
        if point_density > 50000
            % High density: use smaller patch sizes
            inputs.PatchDiam1 = [0.05 0.10];
            inputs.PatchDiam2Min = [0.02 0.04 0.06];
            inputs.PatchDiam2Max = [0.05 0.12];
        else
            % Lower density: use larger patch sizes
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
    
    % (Optional) Run TreeQSM on this single tree for demo
    % QSM = treeqsm(P, inputs);
end

%% Step 3: Batch Preprocessing of Training Point Clouds
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
            filter_inputs.filter.k = 25;         % More neighbors for robust outlier detection
            filter_inputs.filter.nsigma = 3;      % Less strict outlier removal
            filter_inputs.filter.radius = 0;
            filter_inputs.filter.ncomp = 2;       % Keep smaller components
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

%% Step 4: Monte Carlo Parameter Search (Training Set)
num_trials = 2; % Number of random parameter sets to try per tree
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

%% Step 4.5: Compare Training Set DBH with Hand Measurements
% Read hand-measured DBH from Excel
hand_dbh_file = fullfile(pwd, '208_off_DBH.xlsx'); % <-- look in project root
if exist(hand_dbh_file, 'file')
    hand_dbh_data = readtable(hand_dbh_file);
    % Assume columns: TreeID, DBH (cm), adjust as needed
    % Try to find the correct DBH column (case-insensitive)
    dbh_col = find(strcmpi(hand_dbh_data.Properties.VariableNames, 'DBH'), 1);
    if isempty(dbh_col)
        error('Could not find DBH column in 208_off_DBH.xlsx');
    end
    % Merge by TreeID (case-insensitive)
    [lia, locb] = ismember(lower(results_table.TreeID), lower(string(hand_dbh_data.TreeID)));
    measured_dbh = nan(height(results_table),1);
    measured_dbh(lia) = hand_dbh_data{locb(lia), dbh_col};
    results_table.MeasuredDBHcm = measured_dbh;
    % Only compare where measured DBH is available
    valid_idx = ~isnan(measured_dbh);
    est_dbh = results_table.DBHcm(valid_idx);
    true_dbh = measured_dbh(valid_idx);
    % Compute error metrics
    abs_err = abs(est_dbh - true_dbh);
    mae = mean(abs_err);
    rmse = sqrt(mean((est_dbh - true_dbh).^2));
    corrval = corr(est_dbh, true_dbh, 'rows','complete');
    % Print summary
    fprintf('\n=== Training Set DBH Comparison ===\n');
    fprintf('N (with field DBH): %d\n', sum(valid_idx));
    fprintf('MAE:  %.2f cm\n', mae);
    fprintf('RMSE: %.2f cm\n', rmse);
    fprintf('Correlation: %.2f\n', corrval);
    % Optionally print table
    disp(results_table(valid_idx, {'TreeID','Trial','DBHcm','MeasuredDBHcm'}));
else
    warning('Hand-measured DBH file 208_off_DBH.xlsx not found in %s', pwd);
end

%% Step 5: QSM Reconstruction on Selected Tree (Demo)
fprintf('\n========== Running TreeQSM ==========' );

tic;

% Run QSM reconstruction
QSM = treeqsm(P, inputs);

elapsed = toc;
fprintf('QSM reconstruction completed in %.1f seconds\n', elapsed);

%% Step 6: Save & Display QSM Results

% Define and create results directory if it doesn't exist
results_dir = fullfile(pwd, 'results_dir');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

if ~isempty(QSM)
    for k = 1:numel(QSM)
        % Manual save to results directory for each model
        save_filename = sprintf('QSM_%s_t1_m%d.mat', filename, k);
        save_path = fullfile(results_dir, save_filename);
        
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
