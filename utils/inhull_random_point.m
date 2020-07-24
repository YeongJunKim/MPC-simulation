function r = inhull_random_point(p, n)
% p = convex hull point;
% n = return point number;
r = zeros(n,1);

for i = 1:n
    middle = mean(p,1);
    while(1)
        
        nomi = normrnd(middle, 2);
        kkk = convhulln(p);
        in = inhull(nomi, p, kkk);
        if(in == 1)
            r = nomi';
            break;
        end
    end
end