clc;
clear;
close all;

folder=getenv("ROBOT_CODE")+"/camera-calibration";
if ~exist(folder,'dir')
    error('Could not find %s',folder);
end

assets_folder=folder+"/modules/calibSupervisor/app/assets/";
pos_folder=folder+"/modules/calibSupervisor/app/conf/";

img_index=input("Specify index of the image to be visualized: ");
I=imread(assets_folder+"left"+num2str(img_index)+".png");
pos=importdata(pos_folder+"calibrations_gazebo-testmoving.ini");
bb=([pos{img_index}]);

ws=find(isspace(bb)==1);
top=bb(2:ws(4)-2);
bottom=bb(ws(4)+2:ws(8)-2);

ws_top=find(isspace(top)==1);
tl=str2num(top(1:ws_top(2)-1));
tr=str2num(top(ws_top(2)+1:end));

ws_bottom=find(isspace(bottom)==1);
bl=str2num(bottom(1:ws_bottom(2)-1));
br=str2num(bottom(ws_bottom(2)+1:end));

figure(1);
imshow(I);
hold on;
plot([tl(1), tr(1)],[tl(2), tr(2)],'r','Linewidth', 3.0);
plot([tr(1), br(1)],[tr(2), br(2)],'r','Linewidth', 3.0);
plot([br(1), bl(1)],[br(2), bl(2)],'r','Linewidth', 3.0);
plot([bl(1), tl(1)],[bl(2), tl(2)],'r','Linewidth', 3.0);
hold off;