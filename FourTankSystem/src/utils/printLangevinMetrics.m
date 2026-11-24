function printLangevinMetrics(Htrue, Hstat, Hdyn, Dtrue, Dstat, Ddyn, labelStr)
    fprintf('\n=== %s ===\n', labelStr);

    Tank         = (1:4)';
    RMSE_static  = sqrt(mean((Hstat - Htrue).^2, 2));
    RMSE_dynamic = sqrt(mean((Hdyn  - Htrue).^2, 2));
    disp(table(Tank, RMSE_static, RMSE_dynamic));

    Disturbance  = {'F3'; 'F4'};
    RMSE_static  = sqrt(mean((Dstat - Dtrue).^2, 2));
    RMSE_dynamic = sqrt(mean((Ddyn  - Dtrue).^2, 2));
    disp(table(Disturbance, RMSE_static, RMSE_dynamic));
end