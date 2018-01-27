function x = MultiNewtonMethod(of,df,x,blist,plist,Llist,L2)


x=x';
y = of(x,blist,plist,Llist,L2);
dy = df(of,x,blist,plist,Llist,L2);
while 1
    
    s=dy\y;   % inv(dy)*y  dy\y
    x=x-s;
    y = of(x,blist,plist,Llist,L2);
    if y < 0.0001
        break; 
    end
    dy = df(of,x,blist,plist,Llist,L2);
    
end
x=x';
end

