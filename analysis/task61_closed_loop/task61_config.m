function cfg = task61_config()
%TASK61_CONFIG Configuration for homework task 6.1 closed-loop simulation.
%
% The identified plant in this project maps SimpleFOC q-axis voltage Uq [V]
% to mechanical shaft speed omega [rad/s].  Therefore the PI output is
% voltage and the simulation must not multiply by the 12 V supply or an
% additional H-bridge gain again.

thisDirectory = fileparts(mfilename("fullpath"));
analysisDirectory = fileparts(thisDirectory);
projectRoot = fileparts(analysisDirectory);

cfg.projectRoot = projectRoot;
cfg.outputDirectory = fullfile(thisDirectory, "output");

% Prefer the clean four-step acquisition.  The second file is a fallback.
cfg.summaryCandidates = [
    fullfile(projectRoot, "logs", ...
        "motor_step_D_0.1_0.2_0.4_0.6_second_order_b_summary.csv")
    fullfile(projectRoot, "logs", ...
        "device-monitor-261027-152519_second_order_b_summary.csv")
];
cfg.minimumFitPercent = 95.0;

% Safe, previously used operating point for the current 2804 BLDC setup.
cfg.referenceSpeedRadS = 6.28;
cfg.stepTimeS = 0.5;
cfg.stopTimeS = 5.0;
cfg.sampleTimeS = 0.001;

% Matches V1_03_ClosedLoop_Velocity_Test_new initial voltage limit.
cfg.voltageLimitV = 3.0;

% At least two PI groups are required by task 6.1.
% PI-B matches the existing SimpleFOC velocity test. PI-A is a milder group.
cfg.piCases = table( ...
    ["PI-A_mild"; "PI-B_existing"], ...
    [0.08; 0.15], ...
    [0.50; 1.00], ...
    'VariableNames', {'Name', 'Kp', 'Ki'});

cfg.riseLowFraction = 0.10;
cfg.riseHighFraction = 0.90;
cfg.settlingBandFraction = 0.02;
cfg.steadyStateWindowS = 0.50;

cfg.simulinkModelName = "task61_closed_loop_model";
end
