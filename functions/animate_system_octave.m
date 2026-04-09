function animate_system_octave(tin, Xin, traj_din, uin, flag_movie,resampling)
    % Adaptation pour Octave de la fonction animate_system
    % tin : time steps
    % Xin : real state
    % Xin = [x,theta] pos and angle
    % traj_din : desired trajectory
    % uin : control input


    % Préparation de la vidéo (si nécessaire)
    if flag_movie == 1
        try
            name = 'test_pendulum.gif';
            %vidfile = VideoWriter(name, 'MPEG4');
            %vidfile = VideoWriter(name, 'X264'); % 'X264' 'MPEG-4'
            %open(vidfile);
            im = {}; % Tableau de cellules pour stocker les frames
        catch
            warning('Le package "video" n''est pas disponible. L''enregistrement vidéo est désactivé.');
            flag_movie = 0;
        end
    end

    if resampling
      % Pour l'animation
      N_animate = 30;

      % Rééchantillonnage (remplacez even_sample par une interpolation simple si nécessaire)
      t = linspace(tin(1), tin(end), N_animate);
      X = interp1(tin, Xin, t, 'linear');
      traj_d = interp1(tin, traj_din, t, 'linear');
      u = interp1(tin, uin, t, 'linear');
    else
      t = tin;
      X = Xin;
      traj_d = traj_din;
      u = uin;
    end

    % Initialisation de la figure
    figure('Position', [200 100 1000 600]);

    nt = length(t);
    err = zeros(nt, 2);
    traj = zeros(nt, 2);

    l = 0.3;
    lwidth = 3;

    pause(1.0);

    for ii = 1:nt
        %% Animation principale
        subplot(2, 2, 1);
        hold on; grid on; box on;
        plotPendulum_octave(X(ii, :), traj_d(ii, 1));
        title('Pendulum Animation');
        xlabel('x [m]');
        ylabel('y [m]');

        %% Erreur de suivi
        subplot(2, 2, 2);
        hold on; grid on; box on;

        err(ii, :) = traj_d(ii, :) - X(ii, 2); % solely theta error

        % Octave n'a pas yyaxis, donc on utilise deux sous-graphiques superposés
        %[ax, h1, h2] = plotyy(t(1:ii), err(1:ii, 1),t(1:ii), err(1:ii, 2) * 180/pi);
        h1 = plot(t(1:ii), err(1:ii, 1) * 180/pi);
        ylabel('Theta Error [deg]');
        set(h1, 'Color', '#0072BD');
        %set(h2, 'Color', '#D95319');
        %ylabel(ax(1), 'Error [m]');

        title('Tracking Error');
        xlabel('Time [s]');
        %ylim(ax(1), [-1.5 1.5]);
        ylim([-100 100]);

        %% Entrée de contrôle
        subplot(2, 2, 3);
        hold on; grid on; box on;
        plot(t(1:ii), u(1:ii), 'Color', '#0072BD');
        yline = line(0, 0); % ref
        set(yline, 'Color', 'black', 'LineStyle', '--');
        yline_max = line([0,t(ii)], [70,70]);
        set(yline_max, 'Color', 'red', 'LineStyle', '--');
        yline_min = line([0,t(ii)], [-70,-70]);
        set(yline_min, 'Color', 'red', 'LineStyle', '--');
        title('Control Input');
        xlabel('Time [s]');
        ylabel('Motor Voltage [V]');
        %ylim([-30 30]);

        %% Position et angle
        subplot(2, 2, 4);
        hold on; grid on; box on;

        % Octave n'a pas yyaxis, donc on utilise plotyy
        [ax, h1, h2] = plotyy(t(1:ii), X(1:ii, 1), t(1:ii), X(1:ii, 2) * 180/pi);
        set(h1, 'Color', '#0072BD');
        set(h2, 'Color', '#D95319');
        ylabel(ax(1), 'Cart Position [m]');
        ylabel(ax(2), 'Pendulum Angle [deg]');
        title('Cart Position and Angle');
        xlabel('Time [s]');
        ylim(ax(1), [-1.5 1.5]);
        ylim(ax(2), [-100 100]);

        % Ligne pointillée pour la trajectoire souhaitée
        %line(t(1:ii), traj_d(1:ii, 1) * ones(size(t(1:ii))), 'Color', '#0072BD', 'LineStyle', '--');
        line(t(1:ii), traj_d(1:ii, 1) * 180/pi * ones(size(t(1:ii))), 'Color', '#D95319', 'LineStyle', '--');

        % Pause pour l'animation
        pause(0.001);

        % Capture vidéo
        if flag_movie
            frame = getframe(gcf);
            %writeVideo(vidfile, frame);
            im{end+1} = frame.cdata; % Stocke la frame
        end

        % Effacer pour la prochaine itération
        if ii < nt
            clf;
        end
    end

    % Fermer la vidéo
##    if flag_movie
##        close(vidfile);
##    end
    if flag_movie
      for idx = 1:length(im)
        [A_im, map_im] = rgb2ind(im{idx});
        if idx == 1
            imwrite(A_im, map_im, name, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
        else
            imwrite(A_im, map_im, name, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
      end
    disp(['Animation enregistrée sous : ', name]);
    end

end

function plotPendulum_octave(X, traj_d)
    % Fonction pour dessiner le pendule (adaptée pour Octave)

    l = 0.3;
    lwidth = 1.5;

    pos = X(1);
    theta = X(2);

    % Ligne de référence (sol)
    plot([-1, 1], [0, 0], 'Color', 'black', 'LineWidth', lwidth);

    % Masse à l'extrémité du pendule
    circle_center = [pos - l * sin(theta), l * cos(theta)];
    radii = 0.03;

    % Chariot
    rect_w = 0.2;
    rect_h = 0.1;
    rect_x = pos - rect_w/2;
    rect_y = 0;

    rectangle('Position', [rect_x, rect_y, rect_w, rect_h], ...
              'EdgeColor', '#114AA1', 'FaceColor', [0 0.4470 0.7410], 'LineWidth', 1.5*lwidth);

    % Lien (tige du pendule)
    link_x = [pos, circle_center(1)];
    link_y = [rect_y + rect_h, circle_center(2)];
    plot(link_x, link_y, 'Color', '#303030', 'LineWidth', 1.5*lwidth);

    % Cercle pour la masse du pendule
    th = 0:pi/50:2*pi;
    xunit = radii * cos(th) + circle_center(1);
    yunit = radii * sin(th) + circle_center(2);
    fill(xunit, yunit, [0.8500 0.3250 0.0980]);

    % Ligne pointillée pour la trajectoire souhaitée
    %line([traj_d], [0, 0.5], 'Color', 'black', 'LineWidth', 1.5, 'LineStyle', '--');

    axis equal;
    %ylim([-0.1 0.5]);
    xlabel('x [m]');
    ylabel('y [m]');
end
