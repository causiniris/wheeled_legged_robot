function cfg = task62_config()
%TASK62_CONFIG Configuration for homework task 6.2 root-locus analysis.

thisDirectory = fileparts(mfilename("fullpath"));
analysisDirectory = fileparts(thisDirectory);
projectRoot = fileparts(analysisDirectory);

cfg.projectRoot = projectRoot;
cfg.task61Directory = fullfile(analysisDirectory, "task61_closed_loop");
cfg.outputDirectory = fullfile(thisDirectory, "output");

% The same reference and voltage limit used by task 6.1.
cfg.referenceSpeedRadS = 6.28;
cfg.stepTimeS = 0.5;
cfg.stopTimeS = 5.0;
cfg.sampleTimeS = 0.001;
cfg.voltageLimitV = 3.0;

cfg.riseLowFraction = 0.10;
cfg.riseHighFraction = 0.90;
cfg.settlingBandFraction = 0.02;
cfg.steadyStateWindowS = 0.50;

% Each cell contains at least three proportional gains. The corresponding
% integral gain and root-locus gain are calculated as:
%   Ki = -z0*Kp
%   KR = b0*Kp
%
% Z5 and Z80 follow the zero-location examples in the course slides. Z1200
% is an additional electrical-pole-near discussion group requested by the
% slides. Disable a group by setting Enabled to false.
cfg.zeroConfigurations = table( ...
    ["Z5_right"; "Z80_left"; "Z1200_electrical"], ...
    [-5.0; -80.0; -1200.0], ...
    {[0.03, 0.06, 0.12]; [0.02, 0.04, 0.08]; [0.001, 0.002, 0.004]}, ...
    [true; true; true], ...
    ["零点位于主导极点实部右侧"; ...
     "零点位于主导极点实部左侧"; ...
     "电气极点附近的补充讨论方案"], ...
    'VariableNames', {'Name', 'ZeroLocation', 'KpValues', 'Enabled', 'Description'});
end
