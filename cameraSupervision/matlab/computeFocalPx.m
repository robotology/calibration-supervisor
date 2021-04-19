function fpx = computeFocalPx(f,width)
%COMPUTEFOCALPX Computes the focal lenght in pixel

center=width/2;
fpx=center*cot(f/2);
end

