clear all

px = [1 2.5 3 4 8];
py = [2 1.5 1 -3 1];


cubicx = cubicS(px, 8);
cubicy = cubicS(py, 8);

plot(px ,py, '*'), hold on
plot(cubicx.p_seq, cubicy.p_seq)