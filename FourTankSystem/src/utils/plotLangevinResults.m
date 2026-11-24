function plotLangevinResults(t, Htrue, Hstat, Hdyn, Dtrue, Dstat, Ddyn, labelStr)
    % Tank levels (states 1-4)
    figure('Color','w');
    for i = 1:4
        subplot(2,2,i);
        plot(t/60, Htrue(i,:), 'k',  'LineWidth', 1.4); hold on;
        plot(t/60, Hstat(i,:), '--', 'LineWidth', 1.2);
        plot(t/60, Hdyn(i,:),  ':',  'LineWidth', 1.4);
        xlabel('Time [min]');
        ylabel(sprintf('h_%d [cm]', i));
        legend('True', 'Stationary KF', 'Dynamic KF', 'Location', 'best');
        grid on;
    end
    sgtitle([labelStr, ' — Tank Levels']);

    % Unmeasured disturbances (states 5-6)
    figure('Color','w');
    for i = 1:2
        subplot(2,1,i);
        plot(t/60, Dtrue(i,:), 'k',  'LineWidth', 1.4); hold on;
        plot(t/60, Dstat(i,:), '--', 'LineWidth', 1.2);
        plot(t/60, Ddyn(i,:),  ':',  'LineWidth', 1.4);
        xlabel('Time [min]');
        ylabel(sprintf('F_%d [cm^3/s]', i+2));
        legend('True', 'Stationary KF', 'Dynamic KF', 'Location', 'best');
        grid on;
    end
    sgtitle([labelStr, ' — Disturbance Estimation']);
end
