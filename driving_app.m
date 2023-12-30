% Script for simulating lidar scans of input base STL scene (generated using
% matlab driving app)
clear all 
close all

% Create a loop for multiple simulations
simulations = 20;
tform_ndt_history = zeros(simulations,3);
NDT_Results = zeros(simulations,3);
grdstp = .2;
NDT_Error = zeros(simulations,3);

% Create the driving scenario and get sensor data
[allData, scenario, sensor] = generateSensorData();

% Extract point clouds from the generated data
ptCloud1 = allData(end).PointClouds; % Assuming only one point cloud is generated

% Scale the point cloud (optional, depending on your requirements)
scalingFactor = 2;
ptCloud1 = pointCloud(ptCloud1.Location * scalingFactor, 'Color', ptCloud1.Color);

% Rotate the point cloud to match your previous code
theta = -20 / 180 * pi;
rotMat = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];

% Create a 4x4 transformation matrix with rotation
affineMatrix = eye(4);
affineMatrix(1:3, 1:3) = rotMat;

% Apply the rotation using affine3d
tform = affine3d(affineMatrix);
ptCloud1 = pctransform(ptCloud1, tform);

% Visualize the point cloud
figure;
pcshow(ptCloud1);

% Main loop for simulations
for i = 1: simulations
    % Initialize lidar unit
    SensorIndex = 1;
    sensor = monostaticLidarSensor(SensorIndex);
    sensor.HasNoise = false;
    sensor.MountingLocation = [0, 0, 0]; 

    % Set parameters of virtual lidar unit to match Velodyne VLP-16
    sensor.UpdateRate = 10;
    sensor.ElevationLimits = [-22, 10]; 
    sensor.RangeAccuracy = 0.02;
    sensor.AzimuthResolution = 0.35;
    sensor.ElevationResolution = 0.4; 
    sensor.MaxRange = 15;

    % Create a tracking scenario. Add an ego platform and a target platform.
    scenario = trackingScenario;
    ego = platform(scenario, 'Position', [0, 0, 0]);
    target = platform(scenario,'Trajectory',kinematicTrajectory('Position',[0 0 3],'Velocity',[10 10 0]));

    % Specify bounding box for test environment
    target.Dimensions.Length = 25; 
    target.Dimensions.Width = 25;
    target.Dimensions.Height = 11;

    % Obtain the mesh of the target viewed from the ego platform after advancing the scenario
    advance(scenario);
    tgtMeshes = targetMeshes(ego);

    % Obtain the mesh of the target viewed from the ego platform after advancing the scenario
    advance(scenario);
    tgtmeshes = targetMeshes(ego);
    
    % Use the created sensor to generate point clouds from the obtained target mesh.
    time = scenario.SimulationTime;
    [ptCloud1, config, clusters] = sensor(tgtmeshes, time);
    blue_cloud = target.Position;
    blue_angle = target.Orientation;

    % Repeat for the 2nd scan
    advance(scenario);
    tgtmeshes = targetMeshes(ego);
    time = scenario.SimulationTime;
    [ptCloud2, config, clusters] = sensor(tgtmeshes, time);
    orange_cloud = target.Position;
    orange_angle = target.Orientation;

    % Initialize NDT
    relpos = orange_cloud - blue_cloud;
    relangle = orange_angle;
    X_Y_Yaw = [relpos(1), relpos(2), deg2rad(relangle(1))];

    % Register two clouds using NDT
    groundIndx1 = find(ptCloud1(:,3) < 0);
    ptCloud1(groundIndx1, :) = [];
    groundIndx2 = find(ptCloud2(:,3) < 0);
    ptCloud2(groundIndx2, :) = [];
    movingCloud = pointCloud(ptCloud2);
    staticCloud = pointCloud(ptCloud1);
    gridstep = 0.1;

    % Add an initial guess 
    alpha = deg2rad(relangle(1));
    rotationM = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
    tformguess = rigid3d((rotationM)^-1, -relpos);
    tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "InitialTransform", tformguess);
    
    tform_icp = pcregistericp(movingCloud, staticCloud);

    % Store results and errors
    tform_ndt_history(i,:) = tform_ndt.Translation;
    NDT_Results(i,:) = [tform_ndt.Translation(1), tform_ndt.Translation(2), -acos(tform_ndt.R(1,1))];
    NDT_Error(i,:) = X_Y_Yaw + NDT_Results(i,:);

    % Plot translation error
    figure(2); 
    hold on;
    plot(i, abs(NDT_Error(i,1)), '-bo');
    plot(i, abs(NDT_Error(i,2)), '-ro');
    title('Translation Error vs. X Translation');
    xlabel('i');
    ylabel('Error');
    legend('X (m)'); 
    drawnow; % Update the plot

    % Update the 'target' reference to the new 'target' object
    target = new_target;

end

% Placing own rotations and translations on pt cloud
theta = -20/180 * pi;
myrotmat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
my_translation = [0 -2 0];

% Creating third point cloud - matching to first 
ptCloud3 = ptCloud2;
ptCloud3 = (ptCloud3 + tform_ndt.Translation) * (tform_ndt.R)^-1 ; %Trans+Rot ndt

% Visualization
figure()
hold on
axis equal
plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')
plot3(ptCloud3(:,1),ptCloud3(:,2),ptCloud3(:,3),'.')

% Remove all NaNs
ptCloud1 = rmmissing(ptCloud1);
ptCloud2 = rmmissing(ptCloud2);
ptCloud3 = rmmissing(ptCloud3);

figure()
hold on;
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3), 'ro', 'MarkerSize', 5);  % Starting points in red
hold on;
plot3(ptCloud3(:,1),ptCloud3(:,2),ptCloud3(:,3), 'go', 'MarkerSize', 5);  % Ending points (corrected) in green
title('Starting Points vs. Ending Points (NDT Correction)');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Starting Points', 'Ending Points (Corrected)');
grid on;
hold off;

figure()
view(5,10)
hold on;
scatter3(ptCloud2(:,1), ptCloud2(:,2), ptCloud2(:,3), '.', 'b');
hold on;
scatter3(ptCloud3(:,1), ptCloud3(:,2), ptCloud3(:,3), '.', 'g');
title('Starting Points vs. Ending Points (NDT Correction)');
xlabel('X');
ylabel('Y');
zlabel('Z');
z_min = -2.2;  % Minimum Z value
z_max = -.2;   % Maximum Z value
legend('Starting Points', 'Ending Points (Corrected)');
grid on;
hold off;

