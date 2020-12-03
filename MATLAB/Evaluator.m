clear all;                                                                  % Clear variables

myDir = uigetdir;                           									% Path to mesh files
myFiles = dir(fullfile(myDir));                                                       % Get all obj files in struct

mode = '';
wrongFiles = []; 
for i = 1:length(myFiles)                                                   % Determine the file type
    baseFileName = myFiles(i).name; 
    if contains(baseFileName, 'obj')
        mode = 'obj';
        break
    elseif contains(baseFileName, 'pcd')
        mode = 'pcd';
        break
    elseif contains(baseFileName, 'ply')
        mode = 'ply';
        break
    else
        wrongFiles = [wrongFiles i]
    end
end

for n = 1:length(wrongFiles)                                        % Iterate through the indices of elements to remove
   myFiles(wrongFiles(n)) = [];                                          % Remove element at the specified index 
   wrongFiles = wrongFiles - 1;                                 % Decrement indices to reflect the decreased array size
end

mode
if isempty(mode)
    error('No files of the appropriate format found in the selected folder.')
end

lengths = zeros(size(myFiles,1),1);                                         % Allocate results vector

fig = figure('Name','Results','NumberTitle','off','Units', 'pixels');       % Create figure
set(fig, 'visible', 'off');                                                 % Turn off figure's visibility
set(fig, 'Position', [0 0 1100 5000]);                                      % Set figure size
set(fig, 'PaperUnits', 'centimeters');                                      % Set paper units
set(fig, 'Papersize', [29.7 350]);                                          % Set paper size
set(fig, 'PaperPositionMode', 'manual');                                    % Set position mode to manual
set(fig, 'PaperPosition', [0 -40 29.7 400]);                                % Set paper position

for i = 1:length(myFiles)                                                   % Iterate through the mesh files
    baseFileName = myFiles(i).name;                                         % Get the base filename
    fullFileName = fullfile(myDir, baseFileName);                           % Construct the full filename with path
    fprintf(1, 'Now reading %s\n', fullFileName);                           % Output currently processed filename    
    
    switch mode
        case 'obj'
            obj = readObj(fullFileName);                                    % Read OBJ file
            v = obj.v;                                                      % Get point coordinates
            ptCloud = pointCloud(v);                                                % Construct pointcloud from coordinates
            vX = v(:,1);                                                            % Get coordinate vectors
            vY = v(:,2);
            vZ = v(:,3);
        case 'pcd'
            ptCloud = pcread(fullFileName);
            v = ptCloud.Location;
            ptCloud = pointCloud(v);                                                % Construct pointcloud from coordinates
            vX = -v(:,2);                                                            % Get coordinate vectors
            vY = v(:,3);
            vZ = -v(:,1);
        case 'ply'
            ptCloud = pcread(fullFileName);
            v = ptCloud.Location;
            ptCloud = pointCloud(v);                                                % Construct pointcloud from coordinates
            vX = v(:,2);                                                            % Get coordinate vectors
            vY = -v(:,1);
            vZ = v(:,3);
        otherwise
           error('Bad file type') 
    end
    
    % Remove points outside the plane
    removeElements = [];                                                    % Vector to store indices of elements to remove
    for m = 1:length(vX)                                                    % Iterate through points
       if vZ(m) < -2                                                        % If z-component is smaller than -2 (assumed to be outside of the measurement plane)
           removeElements = [removeElements m];                             % Add current index to the removeElements array
       end
    end

    for n = 1:length(removeElements)                                        % Iterate through the indices of elements to remove
       vX(removeElements(n)) = [];                                          % Remove element at the specified index 
       vY(removeElements(n)) = []; 
       vZ(removeElements(n)) = [];
       removeElements = removeElements - 1;                                 % Decrement indices to reflect the decreased array size
    end
    
    ptCloud = pointCloud([vX vY vZ]);                                       % Construct the reduced pointcloud
        
    % Rotate around z
    subplot(length(myFiles), 2, 1+2*(i-1))                                  % Create subplot for the 2D projection
    hold on                                                                 % Hold for plotting multiple times
    plot(vY, vX, '.')
    xlabel('y [m]') 
    ylabel('x [m]')                                                        % Plot the projection
    
    points= [vY vX];                                                        % Get 2D points for line fitting
    
    switch mode
        case 'obj'
            sampleSize = 2;                                                         % Set number of points to sample per trial for RANSAC
            maxDistance = 0.10;                                                     % Set max allowable distance for inliers for RANSAC
        case 'pcd'
            sampleSize = 2; % number of points to sample per trial
            maxDistance = 0.10; % max allowable distance for inliers
        case 'ply'
            sampleSize = 100; % number of points to sample per trial
            maxDistance = 0.05; % max allowable distance for inliers
        otherwise
           error('Bad file type') 
    end

    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1);              % Fit function using polyfit
    evalLineFcn = ...                                                       % Specify distance evaluation function
        @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
    
    [modelRANSAC, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...    % RANSAC
        sampleSize,maxDistance, 'MaxNumTrials', 50);
    
    modelInliers = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);      % Model for the inliers
    
    inlierPts = points(inlierIdx,:);                                        % Select inlier points
    xs = [min(inlierPts(:,1)) max(inlierPts(:,1))];                         % Points to plot the fitted line
    ys = modelInliers(1)*xs + modelInliers(2);
    hold on
    plot(xs, ys, 'g-')                                                      % Plot the fitted line
    axis equal                                                              % Equal scaling on both axes
    hold off
    leg1 = legend({'Points', 'Fitted line'}, 'Location','northoutside');
    leg1.ItemTokenSize = [2,9];

    
     %}, 'Location','none','position',[0 0 0.4 0.2]
    
    gamma = atan(modelInliers(1));                                          % Get the angle of the fitted line
    dY = -modelInliers(2);                                                  % Calculate vertical translation
    rot =   [cos(gamma) -sin(gamma) 0; ...                                  % Compose rotation matrix arount Z
        sin(gamma)  cos(gamma) 0; ...
        0          0     1     ];
    trans = [0, dY, 0];                                                     % Compose translation matrix along Y
    tformZ = rigid3d(rot,trans);                                            % Create the transformation
    
    ptCloud = pctransform(ptCloud,tformZ);                                  % Do the transformation
    v = ptCloud.Location;                                                   % Get coordinates from the transformed pointcloud
    % ptCloud = pointCloud(v);
    
    vX = v(:,1);                                                            % Get coordinate vectors
    vY = v(:,2);
    vZ = v(:,3);
    
    switch mode
        case 'obj'
            [myValues myEdges] = hist(vY,1000);                                     % Histogram along Y
        case 'pcd'
            [myValues myEdges] = hist(vY,1000);                                     % Histogram along Y
        case 'ply'
            [myValues myEdges] = hist(vY,10000);                                     % Histogram along Y
        otherwise
           error('Bad file type') 
    end
        
    subplot(length(myFiles), 2, 2+2*(i-1))                                  % Subplot for the histogram
    hold on
    plot(myEdges, myValues)
    xlabel('x [m]') 
    ylabel('Nr. of points [−]')                                                 % Plot the histogram
    
    % Calculate threshold
    % Calculate the mean of non-zero elements
    summa = 0;
    counter = 0;
    for h = 1:length(myValues)
        if myValues(h) > 0
            summa = summa + myValues(h);
            counter = counter +1;
        end
    end
    
    switch mode
        case 'obj'
            threshold = summa / counter
        case 'pcd'
            threshold = 0.7 * summa / counter
        case 'ply'
            threshold = 1.2 * summa / counter
        otherwise
           error('Bad file type') 
    end    

    for j = 1:length(myValues)
                                                                            % Iterate through the histogram values
        if myValues(j) > threshold                                          % If the current value is above the threshold, found the lower limit
            mini = myEdges(j);                                              % Get the lower limit from the histogram edges
        end
    end
    
    % Do the same for the upper limit
    for k = length(myValues):-1:1        
        if myValues(k) > threshold
            maxi = myEdges(k);
        end
    end
    
    % Plot the limits on the histogram
    xt = [mini maxi];
    hold on;
    plot(xt, [threshold threshold], 'r-')
    hold off;
    leg2 = legend({'Distribution', 'Threshold'}, 'Location','northoutside');
    leg2.ItemTokenSize = [2,9];

    lengths(i) = mini - maxi;                                               % Store the measured distance in the results vector
end
technique = ''
switch mode
    case 'obj'
        technique = 'ARCore'
    case 'pcd'
        technique = 'RealSense_RTAB_Map'
    case 'ply'
        technique = 'RealSense_Single'
    otherwise
       error('None') 
end
resultsFilename = append('Plots_', technique, '.pdf')
saveas(gcf, resultsFilename)                                                    % Save plot to pdf
lengths                                                                     % Display values