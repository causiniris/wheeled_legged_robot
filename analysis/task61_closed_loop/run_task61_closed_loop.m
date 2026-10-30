function results = run_task61_closed_loop()
%RUN_TASK61_CLOSED_LOOP Run task 6.1 PI closed-loop comparisons.
%
% Outputs are written to analysis/task61_closed_loop/output:
%   task61_metrics.csv
%   task61_simulation_data.csv
%   task61_speed_response.png
%   task61_control_voltage.png
%   task61_model_summary.txt
%   task61_results.mat

cfg = task61_config();
model = load_task61_identified_model(cfg);

if ~isfolder(cfg.outputDirectory)
    mkdir(cfg.outputDirectory);
end

time = (0:cfg.sampleTimeS:cfg.stopTimeS)';
reference = cfg.referenceSpeedRadS .* double(time >= cfg.stepTimeS);
numberOfCases = height(cfg.piCases);
speed = zeros(numel(time), numberOfCases);
controlVoltage = zeros(numel(time), numberOfCases);

metrics = table('Size', [numberOfCases, 10], ...
    'VariableTypes', ["string", repmat("double", 1, 9)], ...
    'VariableNames', { ...
        'Name', 'Kp', 'Ki', 'rise_time_s', 'settling_time_s', ...
        'overshoot_percent', 'steady_state_error_rad_s', ...
        'peak_speed_rad_s', 'peak_voltage_V', 'saturation_time_percent'});

for caseIndex = 1:numberOfCases
    Kp = cfg.piCases.Kp(caseIndex);
    Ki = cfg.piCases.Ki(caseIndex);
    [speed(:, caseIndex), controlVoltage(:, caseIndex)] = ...
        simulateClosedLoop(time, model, Kp, Ki, cfg);

    caseMetrics = calculateMetrics( ...
        time, speed(:, caseIndex), controlVoltage(:, caseIndex), cfg);
    metrics(caseIndex, :) = { ...
        cfg.piCases.Name(caseIndex), Kp, Ki, ...
        caseMetrics.riseTimeS, caseMetrics.settlingTimeS, ...
        caseMetrics.overshootPercent, caseMetrics.steadyStateErrorRadS, ...
        caseMetrics.peakSpeedRadS, caseMetrics.peakVoltageV, ...
        caseMetrics.saturationTimePercent};
end

metricsFile = fullfile(cfg.outputDirectory, "task61_metrics.csv");
writetable(metrics, metricsFile);

simulationData = table(time, reference, ...
    'VariableNames', {'time_s', 'reference_rad_s'});
for caseIndex = 1:numberOfCases
    validName = matlab.lang.makeValidName(cfg.piCases.Name(caseIndex));
    simulationData.(validName + "_speed_rad_s") = speed(:, caseIndex);
    simulationData.(validName + "_control_voltage_V") = ...
        controlVoltage(:, caseIndex);
end
simulationDataFile = fullfile( ...
    cfg.outputDirectory, "task61_simulation_data.csv");
writetable(simulationData, simulationDataFile);

speedFigureFile = fullfile( ...
    cfg.outputDirectory, "task61_speed_response.png");
speedFigure = figure("Visible", "off", "Color", "w", ...
    "Name", "Task 6.1 speed response");
plot(time, reference, "k--", "LineWidth", 1.2, ...
    "DisplayName", "reference");
hold on;
colors = lines(numberOfCases);
for caseIndex = 1:numberOfCases
    plot(time, speed(:, caseIndex), "LineWidth", 1.4, ...
        "Color", colors(caseIndex, :), ...
        "DisplayName", sprintf("%s: Kp=%.3g, Ki=%.3g", ...
        cfg.piCases.Name(caseIndex), cfg.piCases.Kp(caseIndex), ...
        cfg.piCases.Ki(caseIndex)));
end
grid on;
xlabel("Time (s)");
ylabel("Shaft speed (rad/s)");
title("Task 6.1 closed-loop speed step response");
legend("Location", "best");
legend("Interpreter", "none");
exportgraphics(speedFigure, speedFigureFile, "Resolution", 180);
close(speedFigure);

voltageFigureFile = fullfile( ...
    cfg.outputDirectory, "task61_control_voltage.png");
voltageFigure = figure("Visible", "off", "Color", "w", ...
    "Name", "Task 6.1 control voltage");
hold on;
for caseIndex = 1:numberOfCases
    plot(time, controlVoltage(:, caseIndex), "LineWidth", 1.3, ...
        "Color", colors(caseIndex, :), ...
        "DisplayName", sprintf("%s Uq", cfg.piCases.Name(caseIndex)));
end
yline(cfg.voltageLimitV, "k--", "HandleVisibility", "off");
yline(-cfg.voltageLimitV, "k--", "HandleVisibility", "off");
grid on;
xlabel("Time (s)");
ylabel("PI output Uq (V)");
title("Task 6.1 PI output and voltage saturation");
legend("Location", "best");
legend("Interpreter", "none");
exportgraphics(voltageFigure, voltageFigureFile, "Resolution", 180);
close(voltageFigure);

summaryFile = fullfile( ...
    cfg.outputDirectory, "task61_model_summary.txt");
writeModelSummary(summaryFile, cfg, model, metrics);

resultsFile = fullfile(cfg.outputDirectory, "task61_results.mat");
save(resultsFile, "cfg", "model", "metrics", "simulationData");

fprintf("Task 6.1 completed.\n");
fprintf("Identification summary: %s\n", model.sourceSummaryFile);
fprintf("Plant: omega/Uq = %.9g / (s^2 + %.9g s + %.9g)\n", ...
    model.b0, model.a1, model.a0);
fprintf("Plant poles: %s\n", mat2str(model.poles, 7));
disp(metrics);
fprintf("Outputs: %s\n", cfg.outputDirectory);

results = struct();
results.config = cfg;
results.model = model;
results.metrics = metrics;
results.simulationData = simulationData;
results.outputDirectory = cfg.outputDirectory;
results.metricsFile = metricsFile;
results.simulationDataFile = simulationDataFile;
results.speedFigureFile = speedFigureFile;
results.voltageFigureFile = voltageFigureFile;
results.summaryFile = summaryFile;
results.resultsFile = resultsFile;
end

function [omega, voltage] = simulateClosedLoop(time, model, Kp, Ki, cfg)
% State x = [omega; omega_dot; integral_error].
state = zeros(3, 1);
omega = zeros(size(time));
voltage = zeros(size(time));

for sampleIndex = 1:numel(time)
    omega(sampleIndex) = state(1);
    [~, voltage(sampleIndex)] = stateDerivative( ...
        time(sampleIndex), state, model, Kp, Ki, cfg);

    if sampleIndex == numel(time)
        break;
    end

    stepSize = time(sampleIndex + 1) - time(sampleIndex);
    currentTime = time(sampleIndex);
    k1 = stateDerivative(currentTime, state, model, Kp, Ki, cfg);
    k2 = stateDerivative(currentTime + stepSize / 2, ...
        state + stepSize * k1 / 2, model, Kp, Ki, cfg);
    k3 = stateDerivative(currentTime + stepSize / 2, ...
        state + stepSize * k2 / 2, model, Kp, Ki, cfg);
    k4 = stateDerivative(currentTime + stepSize, ...
        state + stepSize * k3, model, Kp, Ki, cfg);
    state = state + stepSize * (k1 + 2*k2 + 2*k3 + k4) / 6;
end
end

function [derivative, saturatedVoltage] = stateDerivative( ...
        time, state, model, Kp, Ki, cfg)
reference = cfg.referenceSpeedRadS * double(time >= cfg.stepTimeS);
error = reference - state(1);
rawVoltage = Kp * error + Ki * state(3);
saturatedVoltage = min(max(rawVoltage, -cfg.voltageLimitV), ...
    cfg.voltageLimitV);

% Conditional integration anti-windup. Integration is allowed when the
% output is not saturated or when the error drives the output out of
% saturation.
notSaturated = abs(rawVoltage - saturatedVoltage) < 1e-12;
drivesDownFromUpperLimit = ...
    rawVoltage > cfg.voltageLimitV && error < 0;
drivesUpFromLowerLimit = ...
    rawVoltage < -cfg.voltageLimitV && error > 0;
integrate = notSaturated || drivesDownFromUpperLimit || ...
    drivesUpFromLowerLimit;

derivative = [
    state(2)
    -model.a0 * state(1) - model.a1 * state(2) + ...
        model.b0 * saturatedVoltage
    error * double(integrate)
];
end

function metrics = calculateMetrics(time, speed, voltage, cfg)
afterStep = time >= cfg.stepTimeS;
relativeTime = time(afterStep) - cfg.stepTimeS;
response = speed(afterStep);
control = voltage(afterStep);
target = cfg.referenceSpeedRadS;

lowThreshold = cfg.riseLowFraction * target;
highThreshold = cfg.riseHighFraction * target;
lowIndex = find(response >= lowThreshold, 1, "first");
highIndex = find(response >= highThreshold, 1, "first");
if isempty(lowIndex) || isempty(highIndex) || highIndex < lowIndex
    riseTime = NaN;
else
    riseTime = relativeTime(highIndex) - relativeTime(lowIndex);
end

peakSpeed = max(response);
overshoot = max(0, (peakSpeed - target) / max(abs(target), eps) * 100);

settlingBand = cfg.settlingBandFraction * max(abs(target), eps);
outsideBand = find(abs(response - target) > settlingBand, 1, "last");
if isempty(outsideBand)
    settlingTime = 0;
elseif outsideBand < numel(relativeTime)
    settlingTime = relativeTime(outsideBand + 1);
else
    settlingTime = NaN;
end

steadyStateStart = max(relativeTime(end) - cfg.steadyStateWindowS, 0);
steadyStateRows = relativeTime >= steadyStateStart;
steadyStateMean = mean(response(steadyStateRows), "omitnan");
steadyStateError = target - steadyStateMean;

saturationRows = abs(control) >= cfg.voltageLimitV - 1e-9;

metrics = struct();
metrics.riseTimeS = riseTime;
metrics.settlingTimeS = settlingTime;
metrics.overshootPercent = overshoot;
metrics.steadyStateErrorRadS = steadyStateError;
metrics.peakSpeedRadS = peakSpeed;
metrics.peakVoltageV = max(abs(control));
metrics.saturationTimePercent = 100 * mean(saturationRows);
end

function writeModelSummary(fileName, cfg, model, metrics)
fileId = fopen(fileName, "w", "n", "UTF-8");
if fileId < 0
    error("task61:CannotWriteSummary", "Cannot write %s", fileName);
end
cleanupFile = onCleanup(@() fclose(fileId));

fprintf(fileId, "Task 6.1 closed-loop simulation summary\n");
fprintf(fileId, "Generated: %s\n\n", string(datetime("now")));
fprintf(fileId, "Identification summary: %s\n", model.sourceSummaryFile);
fprintf(fileId, "Plant input: SimpleFOC q-axis voltage Uq [V]\n");
fprintf(fileId, "Plant output: shaft speed omega [rad/s]\n");
fprintf(fileId, ...
    "omega/Uq = %.12g / (s^2 + %.12g*s + %.12g)\n", ...
    model.b0, model.a1, model.a0);
fprintf(fileId, "DC gain: %.12g rad/s/V\n", model.dcGainRadSPerV);
fprintf(fileId, "Poles: %s\n\n", mat2str(model.poles, 10));
fprintf(fileId, "Reference: %.6g rad/s\n", cfg.referenceSpeedRadS);
fprintf(fileId, "Step time: %.6g s\n", cfg.stepTimeS);
fprintf(fileId, "Voltage limit: +/- %.6g V\n", cfg.voltageLimitV);
fprintf(fileId, "Sample interval: %.6g s\n\n", cfg.sampleTimeS);

for row = 1:height(metrics)
    fprintf(fileId, ...
        "%s: Kp=%.6g, Ki=%.6g, rise=%.6g s, settle=%.6g s, " + ...
        "overshoot=%.6g%%, steady error=%.6g rad/s, peak Uq=%.6g V\n", ...
        metrics.Name(row), metrics.Kp(row), metrics.Ki(row), ...
        metrics.rise_time_s(row), metrics.settling_time_s(row), ...
        metrics.overshoot_percent(row), ...
        metrics.steady_state_error_rad_s(row), metrics.peak_voltage_V(row));
end
end
