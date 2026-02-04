function [A] = skmatrix(r)
p = inputParser;

addRequired(p,'r',@(x) isnumeric(x) && isvector(x) && numel(x)==3);

%add input parse
parse(p,r);

r = p.Results.r;
A = [0    -r(3)   r(2);
     r(3)   0    -r(1);
    -r(2)  r(1)    0];
end