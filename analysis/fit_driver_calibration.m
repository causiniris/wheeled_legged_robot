function result = fit_driver_calibration(csvFile)
%FIT_DRIVER_CALIBRATION Fit Ua = k*(D*Udc) + b from homework 4.1 data.
%
% CSV columns: D,Udc_V,Ua_V
% Usage: result = fit_driver_calibration("driver_calibration.csv")

arguments
    csvFile (1,1) string = ""
end

if strlength(csvFile) == 0
    [name, folder] = uigetfile("*.csv", "Select driver calibration CSV");
    if isequal(name, 0)
        error("No CSV file selected.");
    end
    csvFile = fullfile(folder, name);
end

T = readtable(csvFile, "VariableNamingRule", "preserve");
required = ["D", "Udc_V", "Ua_V"];
if ~all(ismember(required, string(T.Properties.VariableNames)))
    error("CSV columns must be: D,Udc_V,Ua_V");
end
T = rmmissing(T, "DataVariables", cellstr(required));
if height(T) < 5
    error("At least five valid duty-cycle points are required.");
end
if any(T.D < 0 | T.D > 1 | T.Udc_V <= 0)
    error("D must be in [0,1] and Udc_V must be positive.");
end

x = T.D .* T.Udc_V;
y = T.Ua_V;
p = polyfit(x, y, 1);
yhat = polyval(p, x);
residual = y - yhat;
ssTotal = sum((y - mean(y)).^2);
rSquared = 1 - sum(residual.^2) / max(ssTotal, eps);

[folder, name] = fileparts(csvFile);
summaryFile = fullfile(folder, name + "_fit_summary.csv");
figureFile = fullfile(folder, name + "_fit.png");
pointFile = fullfile(folder, name + "_fit_points.csv");

summary = table(p(1), p(2), rSquared, sqrt(mean(residual.^2)), ...
    'VariableNames', {'k', 'b_V', 'R_squared', 'RMSE_V'});
points = T;
points.D_times_Udc_V = x;
points.Ua_fit_V = yhat;
points.residual_V = residual;
writetable(summary, summaryFile);
writetable(points, pointFile);

fig = figure("Color", "w", "Name", "Driver calibration");
scatter(x, y, 55, "filled", "MarkerFaceColor", [0.15 0.45 0.75]);
hold on;
xLine = linspace(min(x), max(x), 200);
plot(xLine, polyval(p, xLine), "Color", [0.85 0.25 0.15], "LineWidth", 1.5);
grid on;
xlabel("D U_{dc} (V)");
ylabel("Measured U_a (V)");
title(sprintf("U_a = %.6g D U_{dc} %+.6g, R^2 = %.5f", p(1), p(2), rSquared));
legend("measured", "linear fit", "Location", "best");
exportgraphics(fig, figureFile, "Resolution", 180);

disp(summary);
fprintf("Saved calibration figure: %s\n", figureFile);
fprintf("Saved point residuals: %s\n", pointFile);

result = struct();
result.source = csvFile;
result.k = p(1);
result.b = p(2);
result.RSquared = rSquared;
result.RMSE = sqrt(mean(residual.^2));
result.summaryFile = summaryFile;
result.pointFile = pointFile;
result.figureFile = figureFile;
end
