function par=Init(N,d,range)
for pop_index = 1:N
    for par_index = 1:d,
        par(pop_index,par_index)=...
             (rand-0.5)*(range(2,par_index) -range(1,par_index)) +...
            0.5*(range(2,par_index) + range(1,par_index));
    end
end
