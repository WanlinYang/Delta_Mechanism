function y = DeltaoFunction( x,blist,plist,Llist,L2 )
%original function
%12 equations derived from length of upper legs 
%x 3x1 row vector
y = zeros(size(x));
for i=1:3
    y(i) = (x(1)+plist(i,1)-Llist(i,1)-blist(i,1))^2 + ...
    (x(2)+plist(i,2)-Llist(i,2)-blist(i,2))^2 +...
    (x(3)+plist(i,3)-Llist(i,3)-blist(i,3))^2 - L2^2;
end
end

