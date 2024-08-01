function child=Cross_Twopoint(parent,Pc,elitism,bestchrom)
if(nargin<4),
    error(['khong du doi so']);
end;
[N,L]=size(parent);
for p1=1:N,
    if(elitism==1) & (p1==bestchrom)
        child(p1,:)=parent(p1,:);
    else
        if Pc>rand
            p2=p1;
            while p2==p1,
                p2=rand*N;
                p2=p2-rem(p2,1)+1;
            end
            k1=rand*(L-1);
            k1=k1-rem(k1,1)+1;
            k2=k1;
            while k2==k1,
                k2=rand*(L-1);
                k2=k2-rem(k2,1)+1;
            end;
            if k1>k2, t=k2;k2=k1; k1=t; end;

            child(p1,1:k1)=parent(p1,1:k1);
            child(p1,k1+1:k2)=parent(p2,k1+1:k2);
            child(p1,k2+1:L)=parent(p1,k2+1:L);
        else
            child(p1,:)=parent(p1,:);
        end
    end
end
