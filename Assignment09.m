clear;
clc;
close all;

addpath(genpath('MatlabFns'));

mkdir('_output');

    % Reading in images.. if past the first frame, use the Jregistered
    % image from the previous step as pos1, and load a new image into pos2
    
    pos1 = imread('3.jpg');
    pos2 = imread('4.jpg');
    
    % Convert images to grayscale (necessary for Harris corner detection)
    Im1 = rgb2gray(pos1);
    Im2 = rgb2gray(pos2);  

    % SURF portion to help with Stabilization

    Ipts1 = OpenSurf(Im1);
    Ipts2 = OpenSurf(Im2);

    for k = 1:length(Ipts1)
    D1(:,k) = Ipts1(k).descriptor;
end

for k = 1:length(Ipts2)
    D2(:,k) = Ipts2(k).descriptor;
end

BaseLength = length(Ipts1);
SubLength = length(Ipts2);

for i = 1:BaseLength
    subtract = (repmat(D1(:,i), ...
        [1 SubLength]) - D2).^2;
    distance = sum(subtract);
    [SubValue(i) SubIndex(i)] = min(distance);
end

[value, index] = sort(SubValue);

index = index(1:100);

BaseIndex = index;
SubIndex = SubIndex(index);

m1 = floor([[Ipts1(BaseIndex).y]; ...
    [Ipts1(BaseIndex).x]]);

m2 = floor([[Ipts2(SubIndex).y]; ...
    [Ipts2(SubIndex).x]]);

    % RANSAC
    [H, inliers] = ransacfithomography(m1, m2, 0.001);
    
    % Moving = second frame, Fixed = first frame or Jregistered
    fixedPoints = [m1(2,inliers)' m1(1,inliers)'];
    movingPoints = [m2(2,inliers)' m2(1,inliers)'];
     
    % Determining the transform based on the relationship matrices between
    % the coordinates in the two images
    tform = fitgeotrans(movingPoints,fixedPoints,'NonreflectiveSimilarity');
    
    % Image registration (alignment)
    Jregistered = imwarp(pos2,tform,'OutputView',imref2d(size(pos1)));
    falsecolorOverlay = imfuse(pos1,Jregistered);
    blendcolorOverlay = imfuse(pos1,Jregistered,'blend');
    imshow(falsecolorOverlay)
