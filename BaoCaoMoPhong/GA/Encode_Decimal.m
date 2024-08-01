function pop = Encode_Decimal(par,sig,dec)
if(nargin<3),
    error(['Thieu doi so.Cu phap: pop = Encode_Decimal(par,sig,dec)']);
end;

if size(sig) ~=size(dec),
    error(['Doi so sig va sec khong phu hop nhau']);
end;

[N,d]=size(par);

for pop_index = 1:N,
    gene_index=1;
    for par_index=1:d,
        if par (pop_index,par_index)<0;
            pop (pop_index,gene_index)=0;
        else
            pop(pop_index,gene_index)=9;
        end
        gene_index = gene_index +1;
        
        temp(par_index)=abs(par(pop_index,par_index))/10^dec(par_index);
        for count = 1:sig(par_index),
            temp(par_index)=temp(par_index)*10;
            pop(pop_index,gene_index)=temp(par_index)-...
                rem(temp(par_index),1);
            temp(par_index)=temp(par_index)-...
                pop(pop_index,gene_index);
            gene_index=gene_index+1;
        end
    end
end
