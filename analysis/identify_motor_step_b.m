function results = identify_motor_step_b(csvFile, Rphase, Lq, Ke, Kt, Jdatasheet)
%IDENTIFY_MOTOR_STEP_B Fit each voltage-step run and estimate viscous B.
%
% Usage:
%   results = identify_motor_step_b()
%   results = identify_motor_step_b("motor_step_D_0.1_0.2_0.4_0.6.csv")
%   results = identify_motor_step_b(csvFile, 2.55, 0.9e-3, 0.0434, ...
%                                   0.0434, 3.7e-6)
%
% The defaults are nominal values from the supplied 2804 motor datasheet.
% Input is Uq/Ua command voltage [V]; output is shaft speed [rad/s].
% Requires MATLAB System Identification Toolbox.

arguments
    csvFile (1,1) string = ""
    Rphase (1,1) double {mustBePositive} = 2.55
    Lq (1,1) double {mustBePositive} = 0.9e-3
    Ke (1,1) double {mustBePositive} = 0.0434
    Kt (1,1) double {mustBePositive} = 0.0434
    Jdatasheet (1,1) double {mustBePositive} = 3.7e-6
end

if exist("tfest", "file") ~= 2
    error("identify_motor_step_b:MissingToolbox", ...
        "System Identification Toolbox is required for tfest.");
end

if strlength(csvFile) == 0
    [name, folder] = uigetfile("*.csv", "Select the motor step CSV");
    if isequal(name, 0)
        error("No CSV file selected.");
    end
    csvFile = string(fullfile(folder, name));
end
if ~isfile(csvFile)
    error("CSV file not found: %s", csvFile);
end

T = readtable(csvFile, "VariableNamingRule", "preserve");
variables = string(T.Properties.VariableNames);
required = ["t_us", "run", "phase", "omega_rad_s"];
if ~all(ismember(required, variables))
    error("CSV must contain columns: %s", strjoin(required, ", "));
end

if ismember("uq_cmd_V", variables)
    uRaw = double(T.uq_cmd_V);
    inputColumn = "uq_cmd_V";
elseif ismember("Ua_V", variables)
    uRaw = double(T.Ua_V);
    inputColumn = "Ua_V";
elseif all(ismember(["duty_D", "Udc_V"], variables))
    uRaw = double(T.duty_D) .* double(T.Udc_V);
    inputColumn = "duty_D*Udc_V";
else
    error("CSV needs uq_cmd_V, Ua_V, or both duty_D and Udc_V columns.");
end

valid = isfinite(double(T.t_us)) & isfinite(double(T.run)) & ...
        isfinite(double(T.phase)) & isfinite(uRaw) & ...
        isfinite(double(T.omega_rad_s));
T = T(valid, :);
uRaw = uRaw(valid);
if isempty(T)
    error("CSV contains no valid numeric samples.");
end

runNumbers = unique(double(T.run(:)))';
runNumbers = runNumbers(isfinite(runNumbers));
if isempty(runNumbers)
    error("No valid run numbers found in CSV.");
end

fitOptions = tfestOptions;
fitOptions.Display = "off";
fitOptions.EnforceStability = true;

numberOfRuns = numel(runNumbers);
models = cell(numberOfRuns, 1);
timeSets = cell(numberOfRuns, 1);
outputSets = cell(numberOfRuns, 1);
modelOutputSets = cell(numberOfRuns, 1);
summaryValues = nan(numberOfRuns, 15);

for k = 1:numberOfRuns
    runNo = runNumbers(k);
    phaseValues = double(T.phase);
    rows = double(T.run) == runNo & (phaseValues == 0 | phaseValues == 1);
    R = T(rows, :);
    runInput = uRaw(rows);
    [~, order] = sort(double(R.t_us));
    R = R(order, :);
    runInput = runInput(order);

    phaseValues = double(R.phase);
    baseline = phaseValues == 0;
    step = phaseValues == 1;
    if nnz(baseline) < 20 || nnz(step) < 100
        warning("Run %g skipped: not enough baseline or step samples.", runNo);
        continue;
    end

    t = (double(R.t_us) - double(R.t_us(1))) * 1e-6;
    [t, uniqueIndex] = unique(t, "stable");
    R = R(uniqueIndex, :);
    runInput = runInput(uniqueIndex);
    phaseValues = double(R.phase);
    baseline = phaseValues == 0;
    step = phaseValues == 1;
    y = double(R.omega_rad_s);

    uBaseline = median(runInput(baseline), "omitnan");
    uStep = median(runInput(step), "omitnan");
    if ~isfinite(uStep - uBaseline) || abs(uStep - uBaseline) < 1e-6
        warning("Run %g skipped: step input is zero or invalid.", runNo);
        continue;
    end

    u = runInput - uBaseline;
    y = y - mean(y(baseline), "omitnan");
    sampleTime = median(diff(t));
    if ~isfinite(sampleTime) || sampleTime <= 0
        warning("Run %g skipped: invalid sample interval.", runNo);
        continue;
    end

    % tfest requires uniformly sampled data; interpolate onto the median interval.
    tq = (0:sampleTime:t(end))';
    uq = interp1(t, u, tq, "previous", "extrap");
    yq = interp1(t, y, tq, "linear", "extrap");
    data = iddata(yq, uq, sampleTime, "TimeUnit", "s", ...
        "InputName", "Uq", "InputUnit", "V", ...
        "OutputName", "omega", "OutputUnit", "rad/s");

    try
        discreteModel = tfest(data, 2, 0, fitOptions);
        if discreteModel.Ts ~= 0
            fittedModel = d2c(discreteModel, "zoh");
        else
            fittedModel = discreteModel;
        end
    catch fitError
        warning("Run %g fit failed: %s", runNo, fitError.message);
        continue;
    end

    [numerator, denominator] = tfdata(tf(fittedModel), "v");
    scale = denominator(1);
    numerator = numerator / scale;
    denominator = denominator / scale;
    numeratorTolerance = max(abs(numerator)) * 1e-10;
    firstNumeratorTerm = find(abs(numerator) > numeratorTolerance, 1, "first");
    if isempty(firstNumeratorTerm)
        warning("Run %g skipped: fitted numerator is zero.", runNo);
        continue;
    end
    numerator = numerator(firstNumeratorTerm:end);
    if numel(denominator) ~= 3 || numel(numerator) ~= 1
        warning("Run %g skipped: fitted model is not a 2-pole, 0-zero transfer function.", runNo);
        continue;
    end

    b0 = numerator(1);
    a1 = denominator(2);
    a0 = denominator(3);
    yhat = lsim(fittedModel, uq, tq);
    fitPercent = 100 * (1 - norm(yq - yhat) / ...
        max(norm(yq - mean(yq)), eps));
    dcGain = dcgain(fittedModel);

    Jfit = NaN;
    if b0 > 0
        Jfit = Kt / (Lq * b0);
    end
    BfromDatasheetJ = Jdatasheet * (a1 - Rphase / Lq);
    BfromFitJ = NaN;
    if isfinite(Jfit)
        BfromFitJ = Jfit * (a1 - Rphase / Lq);
    end
    BfromDcGain = NaN;
    if isfinite(dcGain) && dcGain > 0
        BfromDcGain = (Kt / dcGain - Kt * Ke) / Rphase;
    end
    a0FromParameters = ...
        (Rphase * BfromDatasheetJ + Kt * Ke) / (Lq * Jdatasheet);
    a0ErrorPercent = 100 * (a0 - a0FromParameters) / max(abs(a0), eps);

    summaryValues(k, :) = [runNo, uStep - uBaseline, sampleTime, fitPercent, ...
        a1, a0, b0, dcGain, Jfit, Jdatasheet, BfromDatasheetJ, ...
        BfromFitJ, BfromDcGain, a0FromParameters, a0ErrorPercent];
    models{k} = fittedModel;
    timeSets{k} = tq;
    outputSets{k} = yq;
    modelOutputSets{k} = yhat;

    if BfromDatasheetJ <= 0
        warning("Run %g gives non-positive B using datasheet J. Do not report it as a valid " + ...
                "viscous-friction estimate; check the model/parameter conventions and fit range.", runNo);
    end
end

validFits = isfinite(summaryValues(:, 1));
if ~any(validFits)
    error("No runs produced a usable second-order fit.");
end

summary = array2table(summaryValues(validFits, :), "VariableNames", ...
    {'run', 'Uq_step_V', 'Ts_s', 'fit_percent', 'a1_s_inv', 'a0_s2', ...
     'b0', 'dc_gain_rad_s_per_V', 'J_fit_kg_m2', 'J_datasheet_kg_m2', ...
     'B_from_datasheet_J_Nm_s_rad', 'B_from_fit_J_Nm_s_rad', ...
     'B_from_dc_gain_Nm_s_rad', 'a0_from_parameters_s2', 'a0_error_percent'});

[sourceFolder, sourceName] = fileparts(csvFile);
summaryFile = fullfile(sourceFolder, sourceName + "_second_order_b_summary.csv");
figureFile = fullfile(sourceFolder, sourceName + "_second_order_b_fit.png");
writetable(summary, summaryFile);

fig = figure("Visible", "off", "Color", "w", ...
    "Name", "Motor step second-order fits");
layout = tiledlayout(height(summary), 1, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Uq command to angular velocity: per-run second-order fits");
for k = 1:numberOfRuns
    if isempty(models{k})
        continue;
    end
    row = find(summary.run == runNumbers(k), 1);
    nexttile(row);
    plot(timeSets{k}, outputSets{k}, "Color", [0.15 0.45 0.75], "LineWidth", 1.0);
    hold on;
    plot(timeSets{k}, modelOutputSets{k}, "--", "Color", [0.85 0.25 0.15], "LineWidth", 1.2);
    grid on;
    ylabel("omega (rad/s)");
    title(sprintf("Run %g, Uq step %.3g V, fit %.1f%%", ...
        summary.run(row), summary.Uq_step_V(row), summary.fit_percent(row)));
    legend("measured", "2nd-order fit", "Location", "best");
end
xlabel(layout, "Time from baseline start (s)");
exportgraphics(fig, figureFile, "Resolution", 180);
close(fig);

disp(summary);
for row = 1:height(summary)
    fprintf("Run %g fitted transfer function omega/Uq:\n", summary.run(row));
    disp(models{find(runNumbers == summary.run(row), 1)});
    fprintf("B using datasheet J = %.8g N*m*s/rad\n", ...
        summary.B_from_datasheet_J_Nm_s_rad(row));
end
fprintf("Saved fit summary: %s\n", summaryFile);
fprintf("Saved fit plot: %s\n", figureFile);
fprintf("Input column used: %s\n", inputColumn);
fprintf("Nominal parameters: Rs=%.6g ohm, Lq=%.6g H, Ke=%.6g, Kt=%.6g, J=%.6g kg*m^2\n", ...
    Rphase, Lq, Ke, Kt, Jdatasheet);

results = struct();
results.source = csvFile;
results.inputColumn = inputColumn;
results.summary = summary;
results.models = models(validFits);
results.summaryFile = summaryFile;
results.figureFile = figureFile;
results.parameters = struct("Rphase", Rphase, "Lq", Lq, "Ke", Ke, ...
    "Kt", Kt, "Jdatasheet", Jdatasheet);
end
