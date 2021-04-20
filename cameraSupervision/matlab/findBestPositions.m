clc;
clear;
close all;

folder="/home/vvasco/.local/share/yarp/contexts/camera-calibration-best-pos/rgb-cameras/640x480";
if ~exist(folder,'dir')
    error('Could not find %s',folder);
end

ncandidates=input("Specify number of candidates : ");
scores=zeros(ncandidates,1);
for i = 1:ncandidates
    data=importdata(folder+"/candidatePos_"+num2str(i)+"/score.txt");
    if (size(data,1)>1)
        warning("More than one score, something weird happened");
    end
    scores(i)=data(end);
end
[best_score,best_pos]=max(scores);
fprintf('Best position is candidatePos_%i with score %f\n',best_pos,best_score);

[sorted_score,pos]=sort(scores,'descend');
max_range=5;
fprintf('Best first %i positions:\n',max_range);
for i = 1:max_range
    fprintf('%i with score %f\n',pos(i),sorted_score(i));
end


figure(1);
histogram(scores,'BinWidth',0.005,'FaceColor','b');
xlabel('score');

figure(2);
plot(scores,'b');
xlabel('pos index');
ylabel('score');