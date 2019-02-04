function [ cap ] = cap( om )
    cap = zeros(3);
    cap = [     0, -om(3),  om(2);
            om(3),      0, -om(1);
            -om(2), om(1),     0];
end

