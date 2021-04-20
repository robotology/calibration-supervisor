function f = computeFocal(fpx,width)
%COMPUTEFOCAL Computes the focal lenght in radians

center=width/2;
f=2*acot(fpx/center);
end

