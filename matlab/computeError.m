clc;
clear;
close all;

% Create a set of calibration images.
images = imageDatastore(fullfile('/home/vvasco/.local/share/yarp/contexts/cameraCalibration', ...
    'chessboard_centered', 'calibImg_steady'));
imageFileNames = images.Files;

% Detect calibration pattern.
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

% Generate world coordinates of the corners of the squares.
squareSize = 92.41; % millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
I = readimage(images, 1); 
imageSize = [size(I, 1), size(I, 2)];
[params, ~, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'ImageSize', imageSize);

displayErrors(estimationErrors, params);
                                 
%% visualization
figure; 
showExtrinsics(params, 'CameraCentric');
axis equal;

figure; 
showExtrinsics(params, 'PatternCentric');

figure;
showReprojectionErrors(params);