#!/bin/bash
#SBATCH --job-name=matlab_job       # Job name
#SBATCH --output=matlab_job.out     # Standard output and error log
#SBATCH --error=matlab_job.err
#SBATCH --time=36:00:00             # Wall time (hh:mm:ss)
#SBATCH --ntasks=1                  # Number of tasks (usually 1 for MATLAB)
#SBATCH --cpus-per-task=16           # Number of CPU cores per task
#SBATCH --mem=32G                    # Memory per node

# Load the MATLAB module (check available versions with `module avail matlab`)
module load matlab/r2024a

# Run MATLAB in batch mode (no desktop, no splash screen)
matlab -nodisplay -nosplash -r "addpath(genpath(pwd)); run('cyber_system_path_no_plot.m'); exit;"
