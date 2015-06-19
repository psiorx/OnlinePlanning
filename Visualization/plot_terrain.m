% Create a landscape.
[x, y, h, hm, xm, ym] = generate_terrain(7, 513, 0.0, 0.0, 0.0, xlims, ylims);

hm = hm + tree_height/2;

cm = generate_terrain_colors(hm);

figure(25);
surf(xm, ym, max(hm, 0), cm);    % Make figure (and flatten ocean).
set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
axis equal vis3d off;            % Set aspect ratio and turn off axis.
shading interp;                  % Interpolate color across faces.
material dull;                   % Mountains aren't shiny.
camlight left;                   % Add a light over to the left somewhere.
light('Position',[-5 0 -10]);
lighting gouraud; 