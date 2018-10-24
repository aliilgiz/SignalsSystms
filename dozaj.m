function result=dozaj(n)
alfa=100
beta=0.5
    if (n<=1)
        result=alfa
    else
        result=alfa+beta*dozaj(n-1)
    end
end