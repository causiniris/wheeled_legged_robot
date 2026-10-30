function results = run_task62_root_locus()
%RUN_TASK62_ROOT_LOCUS Complete task 6.2 root-locus and step comparisons.
%
% The code uses the identified task 6.1 plant:
%       G(s) = b0 / (s^2 + a1*s + a0)
% and a PI controller:
%       C(s) = Kp + Ki/s = Kp*(s-z0)/s
%
% The normalized root-locus model is:
%       L0(s) = (s-z0) / (s*(s^2+a1*s+a0))
% with variable gain KR = b0*Kp and Ki = -z0*Kp.

cfg = task62_config();
if ~isfolder(cfg.task61Directory)
    error("task62:MissingTask61Code", ...
        "Task 6.1 code directory was not found: %s", cfg.task61Directory);
end
addpath(cfg.task61Directory);
cleanupPath = onCleanup(@() rmpath(cfg.task61Directory));
task61Cfg = task61_config();
plant = load_task61_identified_model(task61Cfg);

if ~isfolder(cfg.outputDirectory)
    mkdir(cfg.outputDirectory);
end

enabledConfigurations = cfg.zeroConfigurations( ...
    cfg.zeroConfigurations.Enabled, :);
if height(enabledConfigurations) < 2
    error("task62:InsufficientZeroConfigurations", ...
        "Task 6.2 requires at least two enabled zero configurations.");
end

totalCases = sum(cellfun(@numel, enabledConfigurations.KpValues));
metrics = createMetricsTable(totalCases);
time = (0:cfg.sampleTimeS:cfg.stopTimeS)';
reference = cfg.referenceSpeedRadS .* double(time >= cfg.stepTimeS);
simulationData = table(time, reference, ...
    'VariableNames', {'time_s', 'reference_rad_s'});

caseRow = 0;
rootLocusFiles = strings(height(enabledConfigurations), 1);
stepResponseFiles = strings(height(enabledConfigurations), 1);
controlVoltageFiles = strings(height(enabledConfigurations), 1);

for configurationIndex = 1:height(enabledConfigurations)
    configurationName = enabledConfigurations.Name(configurationIndex);
    zeroLocation = enabledConfigurations.ZeroLocation(configurationIndex);
    KpValues = enabledConfigurations.KpValues{configurationIndex};
    if numel(KpValues) < 3
        error("task62:InsufficientGains", ...
            "%s has fewer than three Kp/KR work points.", configurationName);
    end

    % L0 denominator is s*(s^2+a1*s+a0).
    normalizedOpenLoop = tf( ...
        [1, -zeroLocation], [1, plant.a1, plant.a0, 0]);
    KRValues = plant.b0 .* KpValues;

    rootLocusFiles(configurationIndex) = plotRootLocus( ...
        normalizedOpenLoop, configurationName, zeroLocation, ...
        KRValues, cfg, plant);

    configurationSpeeds = zeros(numel(time), numel(KpValues));
    configurationVoltages = zeros(numel(time), numel(KpValues));
    caseLabels = strings(numel(KpValues), 1);

    for gainIndex = 1:numel(KpValues)
        caseRow = caseRow + 1;
        Kp = KpValues(gainIndex);
        Ki = -zeroLocation * Kp;
        KR = plant.b0 * Kp;
        caseLabel = sprintf("%s-%d", configurationName, gainIndex);
        caseLabels(gainIndex) = caseLabel;

        openLoop = KR * normalizedOpenLoop;
        closedLoop = feedback(openLoop, 1);
        closedLoopPoles = pole(closedLoop);
        closedLoopPoles = sortPolesByDominance(closedLoopPoles);
        if any(real(closedLoopPoles) >= 0)
            warning("task62:UnstableWorkPoint", ...
                "%s is not stable. Its step metrics will be marked accordingly.", ...
                caseLabel);
        end

        [gainMargin, phaseMargin, gainCrossFrequency, phaseCrossFrequency] = ...
            margin(openLoop);
        gainMarginDb = convertGainMarginToDb(gainMargin);

        [configurationSpeeds(:, gainIndex), ...
         configurationVoltages(:, gainIndex)] = simulateClosedLoop( ...
            time, plant, Kp, Ki, cfg);
        responseMetrics = calculateMetrics( ...
            time, configurationSpeeds(:, gainIndex), ...
            configurationVoltages(:, gainIndex), cfg, closedLoopPoles);

        dominantPole = closedLoopPoles(1);
        dominantNaturalFrequency = abs(dominantPole);
        if dominantNaturalFrequency > 0
            dominantDampingRatio = ...
                -real(dominantPole) / dominantNaturalFrequency;
        else
            dominantDampingRatio = NaN;
        end

        metrics(caseRow, :) = { ...
            string(configurationName), string(caseLabel), zeroLocation, ...
            KR, Kp, Ki, ...
            real(closedLoopPoles(1)), imag(closedLoopPoles(1)), ...
            real(closedLoopPoles(2)), imag(closedLoopPoles(2)), ...
            real(closedLoopPoles(3)), imag(closedLoopPoles(3)), ...
            dominantDampingRatio, dominantNaturalFrequency, ...
            gainMarginDb, phaseMargin, gainCrossFrequency, ...
            phaseCrossFrequency, responseMetrics.riseTimeS, ...
            responseMetrics.settlingTimeS, ...
            responseMetrics.overshootPercent, ...
            responseMetrics.steadyStateErrorRadS, ...
            responseMetrics.peakSpeedRadS, ...
            responseMetrics.peakVoltageV, ...
            responseMetrics.saturationTimePercent, ...
            responseMetrics.isStable};

        validName = matlab.lang.makeValidName(caseLabel);
        simulationData.(validName + "_speed_rad_s") = ...
            configurationSpeeds(:, gainIndex);
        simulationData.(validName + "_control_voltage_V") = ...
            configurationVoltages(:, gainIndex);
    end

    stepResponseFiles(configurationIndex) = plotStepResponses( ...
        time, reference, configurationSpeeds, caseLabels, ...
        KpValues, -zeroLocation .* KpValues, configurationName, ...
        zeroLocation, cfg);
    controlVoltageFiles(configurationIndex) = plotControlVoltages( ...
        time, configurationVoltages, caseLabels, configurationName, cfg);
end

metricsFile = fullfile(cfg.outputDirectory, ...
    "task62_parameter_and_metrics.csv");
writetable(metrics, metricsFile);
simulationDataFile = fullfile(cfg.outputDirectory, ...
    "task62_simulation_data.csv");
writetable(simulationData, simulationDataFile);

summaryFile = fullfile(cfg.outputDirectory, "task62_summary.md");
writeSummaryMarkdown(summaryFile, cfg, plant, enabledConfigurations, metrics, ...
    rootLocusFiles, stepResponseFiles, controlVoltageFiles);

resultsFile = fullfile(cfg.outputDirectory, "task62_results.mat");
save(resultsFile, "cfg", "plant", "metrics", "simulationData", ...
    "rootLocusFiles", "stepResponseFiles", "controlVoltageFiles");

fprintf("Task 6.2 completed.\n");
fprintf("Plant: omega/Uq = %.9g / (s^2 + %.9g s + %.9g)\n", ...
    plant.b0, plant.a1, plant.a0);
fprintf("Normalized root-locus gain relation: KR = %.9g*Kp\n", plant.b0);
disp(metrics(:, { ...
    'Configuration', 'Case', 'z0', 'KR', 'Kp', 'Ki', ...
    'dominant_pole_real', 'dominant_pole_imag', ...
    'rise_time_s', 'settling_time_s', 'overshoot_percent', ...
    'phase_margin_deg', 'stable'}));
fprintf("Outputs: %s\n", cfg.outputDirectory);

results = struct();
results.config = cfg;
results.plant = plant;
results.metrics = metrics;
results.simulationData = simulationData;
results.metricsFile = metricsFile;
results.simulationDataFile = simulationDataFile;
results.summaryFile = summaryFile;
results.resultsFile = resultsFile;
results.rootLocusFiles = rootLocusFiles;
results.stepResponseFiles = stepResponseFiles;
results.controlVoltageFiles = controlVoltageFiles;
end

function metrics = createMetricsTable(numberOfRows)
variableNames = { ...
    'Configuration', 'Case', 'z0', 'KR', 'Kp', 'Ki', ...
    'dominant_pole_real', 'dominant_pole_imag', 'pole2_real', 'pole2_imag', ...
    'pole3_real', 'pole3_imag', 'dominant_damping_ratio', ...
    'dominant_natural_frequency_rad_s', 'gain_margin_dB', ...
    'phase_margin_deg', 'gain_cross_frequency_rad_s', ...
    'phase_cross_frequency_rad_s', 'rise_time_s', ...
    'settling_time_s', 'overshoot_percent', ...
    'steady_state_error_rad_s', 'peak_speed_rad_s', ...
    'peak_voltage_V', 'saturation_time_percent', 'stable'};
variableTypes = ["string", "string", repmat("double", 1, 23), "logical"];
metrics = table('Size', [numberOfRows, numel(variableNames)], ...
    'VariableTypes', variableTypes, 'VariableNames', variableNames);
end

function fileName = plotRootLocus( ...
        normalizedOpenLoop, configurationName, zeroLocation, ...
        KRValues, cfg, plant)
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Name", "Task 6.2 root locus " + configurationName, ...
    "Position", [100 100 1400 600]);
layout = tiledlayout(figureHandle, 1, 2, ...
    "TileSpacing", "compact", "Padding", "compact");

[rootLocations, ~] = rlocus(normalizedOpenLoop);
openLoopPoles = pole(normalizedOpenLoop);
openLoopZeros = zero(normalizedOpenLoop);

workPointPoles = zeros(3, numel(KRValues));
for gainIndex = 1:numel(KRValues)
    characteristic = [1, plant.a1, plant.a0 + KRValues(gainIndex), ...
        -KRValues(gainIndex) * zeroLocation];
    workPointPoles(:, gainIndex) = roots(characteristic);
end

fullAxis = nexttile(layout, 1);
plotRootLocusBranches(fullAxis, rootLocations);
plotOpenLoopMarkers(fullAxis, openLoopPoles, openLoopZeros);
plotSelectedRootLocusPoints(fullAxis, workPointPoles);
grid(fullAxis, "on");
title(fullAxis, "Global root-locus view");
xlabel(fullAxis, "Real axis (s^{-1})");
ylabel(fullAxis, "Imaginary axis (s^{-1})");
applyPaddedLimits(fullAxis, ...
    [rootLocations(:); workPointPoles(:); openLoopPoles(:); openLoopZeros(:)]);

zoomAxis = nexttile(layout, 2);
plotRootLocusBranches(zoomAxis, rootLocations);
plotOpenLoopMarkers(zoomAxis, openLoopPoles, openLoopZeros);
plotSelectedRootLocusPoints(zoomAxis, workPointPoles);
grid(zoomAxis, "on");
title(zoomAxis, "Selected work-point region");
xlabel(zoomAxis, "Real axis (s^{-1})");
ylabel(zoomAxis, "Imaginary axis (s^{-1})");

zoomValues = [workPointPoles(:); plant.poles(:); 0];
applyPaddedLimits(zoomAxis, zoomValues);

title(layout, sprintf("Task 6.2 root locus: %s, z0=%.6g", ...
    configurationName, zeroLocation), "Interpreter", "none");
subtitle(layout, sprintf( ...
    "Reference %.3g rad/s; diamonds 1-%d are selected KR points", ...
    cfg.referenceSpeedRadS, numel(KRValues)));

fileName = fullfile(cfg.outputDirectory, ...
    "task62_root_locus_" + matlab.lang.makeValidName(configurationName) + ".png");
exportgraphics(figureHandle, fileName, "Resolution", 180);
close(figureHandle);
end

function plotRootLocusBranches(axisHandle, rootLocations)
hold(axisHandle, "on");
branchColors = lines(size(rootLocations, 1));
for branchIndex = 1:size(rootLocations, 1)
    plot(axisHandle, real(rootLocations(branchIndex, :)), ...
        imag(rootLocations(branchIndex, :)), ...
        "LineWidth", 1.1, "Color", branchColors(branchIndex, :));
end
end

function plotOpenLoopMarkers(axisHandle, openLoopPoles, openLoopZeros)
plot(axisHandle, real(openLoopPoles), imag(openLoopPoles), "x", ...
    "Color", [0.05 0.35 0.75], "LineWidth", 1.6, "MarkerSize", 8);
plot(axisHandle, real(openLoopZeros), imag(openLoopZeros), "o", ...
    "Color", [0.05 0.05 0.05], "LineWidth", 1.5, "MarkerSize", 8);
end

function plotSelectedRootLocusPoints(axisHandle, workPointPoles)
colors = lines(size(workPointPoles, 2));
for gainIndex = 1:size(workPointPoles, 2)
    selectedPoles = workPointPoles(:, gainIndex);
    plot(axisHandle, real(selectedPoles), imag(selectedPoles), "d", ...
        "MarkerSize", 7, "LineWidth", 1.4, ...
        "Color", colors(gainIndex, :), ...
        "MarkerFaceColor", colors(gainIndex, :));
    for poleIndex = 1:numel(selectedPoles)
        text(axisHandle, real(selectedPoles(poleIndex)), ...
            imag(selectedPoles(poleIndex)), ...
            sprintf("  %d", gainIndex), "Color", colors(gainIndex, :), ...
            "FontSize", 8);
    end
end
end

function applyPaddedLimits(axisHandle, complexValues)
complexValues = complexValues(isfinite(real(complexValues)) & ...
    isfinite(imag(complexValues)));
if isempty(complexValues)
    return;
end
xMinimum = min(real(complexValues));
xMaximum = max(real(complexValues));
xSpan = max(xMaximum - xMinimum, 1);
yMinimum = min(imag(complexValues));
yMaximum = max(imag(complexValues));
ySpan = max(yMaximum - yMinimum, 1);
xlim(axisHandle, [xMinimum - 0.08*xSpan, xMaximum + 0.08*xSpan]);
ylim(axisHandle, [yMinimum - 0.08*ySpan, yMaximum + 0.08*ySpan]);
end

function fileName = plotStepResponses( ...
        time, reference, speeds, caseLabels, KpValues, KiValues, ...
        configurationName, zeroLocation, cfg)
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Name", "Task 6.2 step response " + configurationName);
plot(time, reference, "k--", "LineWidth", 1.2, ...
    "DisplayName", "reference");
hold on;
colors = lines(numel(caseLabels));
for gainIndex = 1:numel(caseLabels)
    plot(time, speeds(:, gainIndex), "LineWidth", 1.35, ...
        "Color", colors(gainIndex, :), ...
        "DisplayName", sprintf("%s: Kp=%.4g, Ki=%.4g", ...
        caseLabels(gainIndex), KpValues(gainIndex), KiValues(gainIndex)));
end
grid on;
xlabel("Time (s)");
ylabel("Shaft speed (rad/s)");
title(sprintf("Task 6.2 step responses: %s, z0=%.6g", ...
    configurationName, zeroLocation), "Interpreter", "none");
legend("Location", "best", "Interpreter", "none");

fileName = fullfile(cfg.outputDirectory, ...
    "task62_step_" + matlab.lang.makeValidName(configurationName) + ".png");
exportgraphics(figureHandle, fileName, "Resolution", 180);
close(figureHandle);
end

function fileName = plotControlVoltages( ...
        time, voltages, caseLabels, configurationName, cfg)
figureHandle = figure("Visible", "off", "Color", "w", ...
    "Name", "Task 6.2 voltage " + configurationName);
hold on;
colors = lines(numel(caseLabels));
for gainIndex = 1:numel(caseLabels)
    plot(time, voltages(:, gainIndex), "LineWidth", 1.2, ...
        "Color", colors(gainIndex, :), ...
        "DisplayName", caseLabels(gainIndex));
end
yline(cfg.voltageLimitV, "k--", "HandleVisibility", "off");
yline(-cfg.voltageLimitV, "k--", "HandleVisibility", "off");
grid on;
xlabel("Time (s)");
ylabel("PI output Uq (V)");
title("Task 6.2 control voltage: " + configurationName, ...
    "Interpreter", "none");
legend("Location", "best", "Interpreter", "none");

fileName = fullfile(cfg.outputDirectory, ...
    "task62_voltage_" + matlab.lang.makeValidName(configurationName) + ".png");
exportgraphics(figureHandle, fileName, "Resolution", 180);
close(figureHandle);
end

function sortedPoles = sortPolesByDominance(poles)
% The pole with the largest real part is the dominant pole.
[~, order] = sortrows([real(poles(:)), abs(imag(poles(:)))], [-1, 1]);
sortedPoles = poles(order);
end

function gainMarginDb = convertGainMarginToDb(gainMargin)
if isinf(gainMargin)
    gainMarginDb = Inf;
elseif gainMargin > 0
    gainMarginDb = 20 * log10(gainMargin);
else
    gainMarginDb = NaN;
end
end

function [omega, voltage] = simulateClosedLoop(time, plant, Kp, Ki, cfg)
state = zeros(3, 1);
omega = zeros(size(time));
voltage = zeros(size(time));

for sampleIndex = 1:numel(time)
    omega(sampleIndex) = state(1);
    [~, voltage(sampleIndex)] = stateDerivative( ...
        time(sampleIndex), state, plant, Kp, Ki, cfg);
    if sampleIndex == numel(time)
        break;
    end

    stepSize = time(sampleIndex + 1) - time(sampleIndex);
    currentTime = time(sampleIndex);
    k1 = stateDerivative(currentTime, state, plant, Kp, Ki, cfg);
    k2 = stateDerivative(currentTime + stepSize/2, ...
        state + stepSize*k1/2, plant, Kp, Ki, cfg);
    k3 = stateDerivative(currentTime + stepSize/2, ...
        state + stepSize*k2/2, plant, Kp, Ki, cfg);
    k4 = stateDerivative(currentTime + stepSize, ...
        state + stepSize*k3, plant, Kp, Ki, cfg);
    state = state + stepSize * (k1 + 2*k2 + 2*k3 + k4) / 6;
end
end

function [derivative, saturatedVoltage] = stateDerivative( ...
        time, state, plant, Kp, Ki, cfg)
reference = cfg.referenceSpeedRadS * double(time >= cfg.stepTimeS);
error = reference - state(1);
rawVoltage = Kp * error + Ki * state(3);
saturatedVoltage = min(max(rawVoltage, -cfg.voltageLimitV), ...
    cfg.voltageLimitV);

notSaturated = abs(rawVoltage - saturatedVoltage) < 1e-12;
drivesDownFromUpperLimit = ...
    rawVoltage > cfg.voltageLimitV && error < 0;
drivesUpFromLowerLimit = ...
    rawVoltage < -cfg.voltageLimitV && error > 0;
integrate = notSaturated || drivesDownFromUpperLimit || ...
    drivesUpFromLowerLimit;

derivative = [
    state(2)
    -plant.a0*state(1) - plant.a1*state(2) + ...
        plant.b0*saturatedVoltage
    error * double(integrate)
];
end

function metrics = calculateMetrics(time, speed, voltage, cfg, poles)
afterStep = time >= cfg.stepTimeS;
relativeTime = time(afterStep) - cfg.stepTimeS;
response = speed(afterStep);
control = voltage(afterStep);
target = cfg.referenceSpeedRadS;

isStable = all(real(poles) < 0) && all(isfinite(response));
if ~isStable
    metrics = struct( ...
        "riseTimeS", NaN, "settlingTimeS", NaN, ...
        "overshootPercent", NaN, "steadyStateErrorRadS", NaN, ...
        "peakSpeedRadS", max(abs(response)), ...
        "peakVoltageV", max(abs(control)), ...
        "saturationTimePercent", 100*mean(abs(control) >= cfg.voltageLimitV-1e-9), ...
        "isStable", false);
    return;
end

lowIndex = find(response >= cfg.riseLowFraction*target, 1, "first");
highIndex = find(response >= cfg.riseHighFraction*target, 1, "first");
if isempty(lowIndex) || isempty(highIndex) || highIndex < lowIndex
    riseTime = NaN;
else
    riseTime = relativeTime(highIndex) - relativeTime(lowIndex);
end

peakSpeed = max(response);
overshoot = max(0, (peakSpeed-target)/max(abs(target), eps)*100);
settlingBand = cfg.settlingBandFraction * max(abs(target), eps);
outsideBand = find(abs(response-target) > settlingBand, 1, "last");
if isempty(outsideBand)
    settlingTime = 0;
elseif outsideBand < numel(relativeTime)
    settlingTime = relativeTime(outsideBand+1);
else
    settlingTime = NaN;
end

steadyStateStart = max(relativeTime(end)-cfg.steadyStateWindowS, 0);
steadyRows = relativeTime >= steadyStateStart;
steadyStateError = target - mean(response(steadyRows), "omitnan");

metrics = struct();
metrics.riseTimeS = riseTime;
metrics.settlingTimeS = settlingTime;
metrics.overshootPercent = overshoot;
metrics.steadyStateErrorRadS = steadyStateError;
metrics.peakSpeedRadS = peakSpeed;
metrics.peakVoltageV = max(abs(control));
metrics.saturationTimePercent = ...
    100*mean(abs(control) >= cfg.voltageLimitV-1e-9);
metrics.isStable = true;
end

function writeSummaryMarkdown( ...
        fileName, cfg, plant, configurations, metrics, ...
        rootLocusFiles, stepResponseFiles, controlVoltageFiles)
fileId = fopen(fileName, "w", "n", "UTF-8");
if fileId < 0
    error("task62:CannotWriteSummary", "Cannot write %s", fileName);
end
cleanupFile = onCleanup(@() fclose(fileId));

fprintf(fileId, "# 任务 6.2 根轨迹与参数对比结果\n\n");
fprintf(fileId, "生成时间：%s\n\n", string(datetime("now")));
fprintf(fileId, "## 对象模型\n\n");
fprintf(fileId, ...
    "辨识模型为 `G(s)=%.9g/(s^2+%.9g s+%.9g)`。\n\n", ...
    plant.b0, plant.a1, plant.a0);
fprintf(fileId, ...
    "PI 控制器为 `C(s)=Kp+Ki/s=Kp(s-z0)/s`，参数关系为 " + ...
    "`Ki=-z0*Kp`、`KR=%.9g*Kp`。\n\n", plant.b0);
fprintf(fileId, "参考速度为 %.6g rad/s，Uq 限幅为 ±%.6g V。\n\n", ...
    cfg.referenceSpeedRadS, cfg.voltageLimitV);

for configurationIndex = 1:height(configurations)
    name = configurations.Name(configurationIndex);
    rows = metrics.Configuration == name;
    fprintf(fileId, "## %s\n\n", name);
    fprintf(fileId, "%s，`z0=%.6g`。\n\n", ...
        configurations.Description(configurationIndex), ...
        configurations.ZeroLocation(configurationIndex));
    fprintf(fileId, "- 根轨迹图：`%s`\n", rootLocusFiles(configurationIndex));
    fprintf(fileId, "- 阶跃曲线：`%s`\n", stepResponseFiles(configurationIndex));
    fprintf(fileId, "- 控制电压：`%s`\n\n", controlVoltageFiles(configurationIndex));
    fprintf(fileId, ...
        "|组别|KR|Kp|Ki|主导极点|上升时间/s|调节时间/s|超调/%%|相位裕度/°|\n");
    fprintf(fileId, "|---|---:|---:|---:|---|---:|---:|---:|---:|\n");
    selected = metrics(rows, :);
    for row = 1:height(selected)
        poleText = formatComplex( ...
            selected.dominant_pole_real(row), ...
            selected.dominant_pole_imag(row));
        fprintf(fileId, ...
            "|%s|%.6g|%.6g|%.6g|%s|%.6g|%.6g|%.6g|%.6g|\n", ...
            selected.Case(row), selected.KR(row), selected.Kp(row), ...
            selected.Ki(row), poleText, selected.rise_time_s(row), ...
            selected.settling_time_s(row), ...
            selected.overshoot_percent(row), selected.phase_margin_deg(row));
    end
    fprintf(fileId, "\n");
end

fprintf(fileId, "## 使用说明\n\n");
fprintf(fileId, ...
    "正式报告应从每类至少选择 3 组参数，保留根轨迹、全部闭环极点、" + ...
    "阶跃响应和超调/调节时间。若老师只要求两类，可提交 Z5 与 Z80，" + ...
    "并把 Z1200 作为电气极点附近的补充讨论。\n");
end

function textValue = formatComplex(realPart, imaginaryPart)
if abs(imaginaryPart) < 1e-10
    textValue = sprintf("%.6g", realPart);
elseif imaginaryPart >= 0
    textValue = sprintf("%.6g+j%.6g", realPart, imaginaryPart);
else
    textValue = sprintf("%.6g-j%.6g", realPart, abs(imaginaryPart));
end
end
