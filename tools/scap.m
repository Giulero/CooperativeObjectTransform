function [ scap ] = scap( skew )
    scap = zeros(3,1);
    scap(1) = skew(3,2);
    scap(2) = skew(1,3);
    scap(3) = skew(2,1);
end

