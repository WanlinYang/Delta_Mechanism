function dy = DeltadFunction(of,x,blist,plist,Llist,L2)

n = length(x);
dy = zeros(n,n);
h = 0.0001;
mp = zeros(size(x));  %matrix minus & plus
for i=1:n
    for j=1:n
        mp(j)=1;
        xp = x+h*mp;
        xm = x-h*mp;
        y = (of(xp,blist,plist,Llist,L2)-...
        of(xm,blist,plist,Llist,L2))/2;
        dy(i,j)=(y(i))/h;
        mp = zeros(size(x));
    end
    mp = zeros(size(x));
end
end


