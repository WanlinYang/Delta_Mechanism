function d = Distance(a,b)
%calculate scalar distance between two points
%   a = [ax,ay,az]   b = [bx,by,bz]
d = sqrt((a(1)-b(1))^2+(a(2)-b(2))^2+(a(3)-b(3))^2);
end

