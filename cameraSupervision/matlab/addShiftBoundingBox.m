clc;
clear;
close all;

folder=getenv('HOME')+"/.local/share/yarp/contexts/camera-calibration-best-pos/event-cameras-last-2/candidatePos_1/";
if ~exist(folder,'dir')
    error('Could not find %s',folder);
end

SHIFT = 20;
WIDTH = 304;
HEIGHT = 240;

fid = fopen(folder+"pos_test.ini",'wt');
for img_index = 1 : 30
    I=imread(folder+"evt"+num2str(img_index)+".png");
    pos=importdata(folder+"pos.ini");
    bb=([pos{img_index}]);
    
    ws=find(isspace(bb)==1);
    top=bb(2:ws(4)-2);
    bottom=bb(ws(4)+2:ws(8)-2);
    
    ws_top=find(isspace(top)==1);
    tl=str2num(top(1:ws_top(2)-1));
    tr=str2num(top(ws_top(2)+1:end));
    tr(1)=tr(1)+SHIFT;
    
    ws_bottom=find(isspace(bottom)==1);
    bl=str2num(bottom(1:ws_bottom(2)-1));
    br=str2num(bottom(ws_bottom(2)+1:end));
    bl(2)=bl(2)+SHIFT;
    br=br+SHIFT;
    
    if(tl(1)>=WIDTH)
        tl(1)=WIDTH-1.5;
    end
    if(tr(1)>=WIDTH)
        tr(1)=WIDTH-1.5;
    end
    if(bl(1)>=WIDTH)
        bl(1)=WIDTH-1.5;
    end
    if(br(1)>=WIDTH)
        br(1)=WIDTH-1.5;
    end
    
    if(tl(2)>=HEIGHT)
        tl(2)=HEIGHT-1.5;
    end
    if(tr(2)>=HEIGHT)
        tr(2)=HEIGHT-1.5;
    end
    if(bl(2)>=HEIGHT)
        bl(2)=HEIGHT-1.5;
    end
    if(br(2)>=HEIGHT)
        br(2)=HEIGHT-1.5;
    end
    
    figure(1);
    imshow(I);
    hold on;
    plot([tl(1), tr(1)],[tl(2), tr(2)],'r','Linewidth', 3.0);
    plot([tr(1), br(1)],[tr(2), br(2)],'r','Linewidth', 3.0);
    plot([br(1), bl(1)],[br(2), bl(2)],'r','Linewidth', 3.0);
    plot([bl(1), tl(1)],[bl(2), tl(2)],'r','Linewidth', 3.0);
    hold off;
    drawnow;
    
    fprintf(fid, '%c%s%c%s%c%s%c%s%s%s%c%s%c%s%c%s%c%s\n',"(",num2str(tl(1))," ",num2str(tl(2)),...
        " ", num2str(tr(1)), " ", num2str(tr(2)),...
        ") (", num2str(bl(1))," ", num2str(bl(2)),...
        " ",num2str(br(1))," ",num2str(br(2)),")",bb(ws(end):end));
end
fclose(fid);


