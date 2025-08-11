close all;
clear;
clc;

% Set the directory containing the files
dataDir = '/Users/gphillip/Documents/MATLAB/coreg_multi_cps/results/n3x3'; % <-- change this

% Find all files starting with "results_summary" in the directory
fileStruct = dir(fullfile(dataDir, 'results_summary*'));

% Convert to full paths
files = fullfile({fileStruct.folder}, {fileStruct.name});

ilp_values = zeros(19,1);
ilpds_values = zeros(19,1);
utilization_values = 0.05:0.05:0.95;
utilization_values = utilization_values';

for j = 1:length(files)

    file = files{j};
    fid = fopen(file, 'r');
    raw = fread(fid, inf);
    fclose(fid);
    str = char(raw');
    data = jsondecode(str);
    
    % Extract keys (decimal values as strings)
    keys = fieldnames(data);
    
    % Convert keys to numeric decimal values
    decimal_values = str2double(keys);
    
    % Preallocate vectors
    ILP_values = zeros(size(decimal_values));
    ILPDS_values = zeros(size(decimal_values));
    
    % Extract ILP and ILPDS values
    for i = 1:length(keys)
        ILP_values(i) = data.(keys{i}).ILP;
        ILPDS_values(i) = data.(keys{i}).ILPDS;
    end
    
    ilp_values = ilp_values+ILP_values;
    ilpds_values = ilpds_values+ILPDS_values;

end

% Display results
figure;
hold on;
plot(utilization_values, ilp_values / length(files), "LineWidth",1);
plot(utilization_values, ilpds_values / length(files), "LineWidth",1);
legend(["ILP", "ILP-DS"])
xlabel("Utilization")
ylabel("Percent Schedulable")
grid on
axis([0 1 0 1])
title("Tasks=3, Modes=3")

hold off;
