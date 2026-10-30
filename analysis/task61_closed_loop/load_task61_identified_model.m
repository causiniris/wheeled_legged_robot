function model = load_task61_identified_model(cfg)
%LOAD_TASK61_IDENTIFIED_MODEL Load a robust median second-order plant model.
%
% Expected fitted form:
%       omega(s)             b0
%       -------- = -----------------------
%         Uq(s)     s^2 + a1*s + a0

% Only stable positive fits above the configured fit threshold are used.

arguments
    cfg (1,1) struct = task61_config()
end

summaryFile = "";
for candidate = reshape(string(cfg.summaryCandidates), 1, [])
    if isfile(candidate)
        summaryFile = candidate;
        break;
    end
end
if strlength(summaryFile) == 0
    error("task61:MissingIdentificationSummary", ...
        "No identification summary CSV was found. Expected one of:\n%s", ...
        strjoin(string(cfg.summaryCandidates), newline));
end

summaryTable = readtable(summaryFile, "VariableNamingRule", "preserve");
requiredColumns = ["fit_percent", "a1_s_inv", "a0_s2", "b0"];
if ~all(ismember(requiredColumns, string(summaryTable.Properties.VariableNames)))
    error("task61:InvalidIdentificationSummary", ...
        "Identification summary is missing columns: %s", ...
        strjoin(requiredColumns, ", "));
end

fitPercent = double(summaryTable.("fit_percent"));
a1Values = double(summaryTable.("a1_s_inv"));
a0Values = double(summaryTable.("a0_s2"));
b0Values = double(summaryTable.("b0"));

valid = isfinite(fitPercent) & isfinite(a1Values) & ...
        isfinite(a0Values) & isfinite(b0Values) & ...
        fitPercent >= cfg.minimumFitPercent & ...
        a1Values > 0 & a0Values > 0 & b0Values > 0;
if ~any(valid)
    error("task61:NoValidIdentificationRuns", ...
        "No stable positive identification run has fit >= %.1f%%.", ...
        cfg.minimumFitPercent);
end

model.a1 = median(a1Values(valid), "omitnan");
model.a0 = median(a0Values(valid), "omitnan");
model.b0 = median(b0Values(valid), "omitnan");
model.numerator = model.b0;
model.denominator = [1, model.a1, model.a0];
model.poles = roots(model.denominator);
model.dcGainRadSPerV = model.b0 / model.a0;
model.sourceSummaryFile = summaryFile;
model.validRuns = summaryTable(valid, :);

if any(real(model.poles) >= 0)
    error("task61:UnstableIdentifiedPlant", ...
        "The selected identified plant is not open-loop stable.");
end
end
