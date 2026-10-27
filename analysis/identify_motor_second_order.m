function results = identify_motor_second_order(csvFile, Rphase, Lq, Ke, Kt)
%IDENTIFY_MOTOR_SECOND_ORDER Fit Uq -> speed second-order models from V1_05 CSV.
%
% Usage:
%   results = identify_motor_second_order("motor_id.csv")
%   results = identify_motor_second_order("motor_id.csv", Rphase, Lq, Ke, Kt)
%
% Units:
%   Rphase [ohm], Lq [H], Ke [V/(rad/s)], Kt [N*m/A].
%   Pass measured phase/axis values if J and B are required. If these four
%   parameters are omitted, the function still fits the black-box model.
%
% Required toolbox: System Identification Toolbox.

arguments
    csvFile (1,1) string = ""
    Rphase (1,1) double = NaN
    Lq (1,1) double = NaN
    Ke (1,1) double = NaN
    Kt (1,1) double = NaN
end

if strlength(csvFile) == 0
    [name, folder] = uigetfile("*.csv", "Select V1_05 identification CSV");
    if isequal(name, 0)
        error("No CSV file selected.");
    end
    csvFile = fullfile(folder, name);
end

rawLines = readlines(csvFile);
headerMask = startsWith(strtrim(rawLines), "t_us,run,phase,uq_cmd_V");
headerIndex = find(headerMask, 1, "last");
if isempty(headerIndex)
    error("No V1_05 CSV header found. Start logging before sending S and keep the CSV text.");
end
if nnz(headerMask) > 1
    warning("Multiple acquisition headers found; the last session will be fitted.");
end
candidateLines = strtrim(rawLines(headerIndex + 1:end));
numericLineMask = ~cellfun("isempty", regexp(cellstr(candidateLines), "^\d+,\d+,\d+,", "once"));
dataLines = candidateLines(numericLineMask);
if isempty(dataLines)
    error("No V1_05 numeric data lines were found after the last CSV header.");
end
temporaryCsv = string(tempname) + ".csv";
writelines([strtrim(rawLines(headerIndex)); dataLines], temporaryCsv);
cleanupTemporaryCsv = onCleanup(@() delete(temporaryCsv));
opts = detectImportOptions(temporaryCsv, "FileType", "text");
opts.VariableNamingRule = "preserve";
T = readtable(temporaryCsv, opts);

required = ["t_us", "run", "phase", "uq_cmd_V", "omega_rad_s", "angle_rad", "dt_us"];
if ~all(ismember(required, string(T.Properties.VariableNames)))
    error("CSV columns do not match V1_05. Required: %s", strjoin(required, ", "));
end

T = rmmissing(T, "DataVariables", cellstr(required));
if isempty(T)
    error("CSV contains no valid numeric samples.");
end

badDt = T.dt_us > 1200 | T.dt_us <= 0;
if any(badDt)
    warning("%d samples have dt_us outside (0, 1200]. Repeat acquisition for formal submission.", nnz(badDt));
end

runNumbers = unique(T.run(:))';
runNumbers = runNumbers(isfinite(runNumbers));
if isempty(runNumbers)
    error("No run numbers found in CSV.");
end

models = cell(numel(runNumbers), 1);
dataSets = cell(numel(runNumbers), 1);
timeSets = cell(numel(runNumbers), 1);
inputSets = cell(numel(runNumbers), 1);
outputSets = cell(numel(runNumbers), 1);
modelOutputSets = cell(numel(runNumbers), 1);

summary = table('Size', [numel(runNumbers), 11], ...
    'VariableTypes', repmat("double", 1, 11), ...
    'VariableNames', ["run", "Uq_V", "Ts_s", "fit_percent", "a1", "a0", "b0", ...
                      "dc_gain", "J_kg_m2", "B_Nm_s_rad", "a0_model_check"]);

fitOptions = tfestOptions;
fitOptions.Display = "off";
fitOptions.EnforceStability = true;

for k = 1:numel(runNumbers)
    runNo = runNumbers(k);
    rows = T.run == runNo & (T.phase == 0 | T.phase == 1);
    R = sortrows(T(rows, :), "t_us");
    if height(R) < 100
        error("Run %g has too few baseline/step samples.", runNo);
    end

    t = double(R.t_us) * 1e-6;
    t = t - t(1);
    [t, uniqueIndex] = unique(t, "stable");
    u = double(R.uq_cmd_V(uniqueIndex));
    y = double(R.omega_rad_s(uniqueIndex));
    phaseValues = double(R.phase(uniqueIndex));

    baseline = phaseValues == 0;
    if nnz(baseline) < 20
        error("Run %g has too few baseline samples.", runNo);
    end
    y = y - mean(y(baseline), "omitnan");

    Ts = median(diff(t));
    if ~(isfinite(Ts) && Ts > 0)
        error("Run %g has an invalid sample interval.", runNo);
    end
    tq = (0:Ts:t(end))';
    uq = interp1(t, u, tq, "previous", "extrap");
    yq = interp1(t, y, tq, "linear", "extrap");

    data = iddata(yq, uq, Ts, "TimeUnit", "s", ...
                  "InputName", "Uq", "InputUnit", "V", ...
                  "OutputName", "omega", "OutputUnit", "rad/s");
    sys = tfest(data, 2, 0, fitOptions);
    if sys.Ts ~= 0
        sys = d2c(sys, "zoh");
    end

    [num, den] = tfdata(tf(sys), "v");
    den = den / den(1);
    num = num / den(1);
    b0 = num(end);
    a1 = den(end - 1);
    a0 = den(end);

    yhat = lsim(tf(sys), uq, tq);
    fitPercent = 100 * (1 - norm(yq - yhat) / max(norm(yq - mean(yq)), eps));
    dcGain = dcgain(sys);

    J = NaN;
    B = NaN;
    a0Check = NaN;
    if all(isfinite([Rphase, Lq, Ke, Kt])) && Rphase > 0 && Lq > 0 && Kt ~= 0 && b0 ~= 0
        J = Kt / (Lq * b0);
        B = J * (a1 - Rphase / Lq);
        a0Check = (Rphase * B + Kt * Ke) / (Lq * J);
    end

    summary{k, :} = [runNo, max(uq), Ts, fitPercent, a1, a0, b0, dcGain, J, B, a0Check];
    models{k} = sys;
    dataSets{k} = data;
    timeSets{k} = tq;
    inputSets{k} = uq;
    outputSets{k} = yq;
    modelOutputSets{k} = yhat;
end

validFits = summary.fit_percent > 0 & isfinite(summary.fit_percent) & ...
            summary.a1 > 0 & summary.a0 > 0 & summary.b0 > 0;
if ~any(validFits)
    warning("No run passed the basic positive/stable-model checks. Inspect wiring, direction and data.");
    consensusModel = tf(NaN, [1 NaN NaN]);
else
    consensusCoefficients = median(summary{validFits, ["a1", "a0", "b0"]}, 1);
    consensusModel = tf(consensusCoefficients(3), [1 consensusCoefficients(1:2)]);
end

[sourceFolder, sourceName] = fileparts(csvFile);
summaryFile = fullfile(sourceFolder, sourceName + "_model_summary.csv");
figureFile = fullfile(sourceFolder, sourceName + "_model_fit.png");
writetable(summary, summaryFile);

fig = figure("Color", "w", "Name", "Motor identification fits");
layout = tiledlayout(numel(runNumbers), 1, "TileSpacing", "compact", "Padding", "compact");
title(layout, "U_q to angular velocity second-order identification");
for k = 1:numel(runNumbers)
    nexttile;
    plot(timeSets{k}, outputSets{k}, "Color", [0.15 0.45 0.75], "LineWidth", 1.0);
    hold on;
    plot(timeSets{k}, modelOutputSets{k}, "--", "Color", [0.85 0.25 0.15], "LineWidth", 1.2);
    grid on;
    ylabel("omega rad/s");
    title(sprintf("Run %g, Uq=%.3g V, fit=%.1f%%", ...
          summary.run(k), summary.Uq_V(k), summary.fit_percent(k)));
    legend("measured", "model", "Location", "best");
end
xlabel(layout, "time s");
exportgraphics(fig, figureFile, "Resolution", 180);

disp(summary);
disp("Consensus model (median coefficients from stable positive runs):");
disp(consensusModel);
fprintf("Saved summary: %s\n", summaryFile);
fprintf("Saved fit figure: %s\n", figureFile);

if all(isfinite([Rphase, Lq, Ke, Kt]))
    if any(summary.J_kg_m2 <= 0 | summary.B_Nm_s_rad <= 0)
        warning("Some J/B estimates are non-positive. Check phase-vs-line R/L, Ke/Kt convention, " + ...
                "sensor velocity filtering, and whether the response is in the linear range.");
    end
else
    fprintf("J and B were not calculated. Re-run with measured Rphase, Lq, Ke and Kt, e.g.\n" + ...
            "identify_motor_second_order(""%s"", Rphase, Lq, Ke, Kt)\n", csvFile);
end

results = struct();
results.source = csvFile;
results.summary = summary;
results.models = models;
results.data = dataSets;
results.consensusModel = consensusModel;
results.summaryFile = summaryFile;
results.figureFile = figureFile;
end
