function [q,v,a] = CubicControlOutputs(t)

    if (t < 1) 
        q = - t^3 + (3*t^2)/2;
        v = - 3*t^2 + 3*t;
        a = 3 - 6*t;
    elseif t < 2 
        q = t^3 - (9*t^2)/2 + 6*t - 2;
        v = 3*t^2 - 9*t + 6;
        a = 6*t - 9;
    else
        q = 0;
        v = 0;
        a = 0;
    end



end