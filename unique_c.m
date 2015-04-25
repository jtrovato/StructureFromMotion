function Cu = unique_c(Cset, Cnew)
 
    for i=1:length(Cset)
        if Cest{i} == Cnew
            return Cset
        end
    end
    Cset{end+1} = Cnew;
    return Cset;

end

