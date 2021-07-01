function [ clock_us ] = getSimClock ( sharedmemory )
    clock_us =  double(typecast( sharedmemory.Data  , 'uint64' ))  ; % get sim clock
end