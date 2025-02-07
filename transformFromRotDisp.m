function H = transformFromRotDisp(R,d)
H = [
R(:,:), d(:)    
0, 0, 0, 1;
];
end
