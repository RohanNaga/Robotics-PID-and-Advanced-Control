function [q,v,a] = CubicControlOutputs(t)

    if (mod(t,1) < .5) 
        q = - t^3 + (3*t^2)/2;
        v = - 3*t^2 + 3*t;
        a = 3 - 6*t;
    else 
        q = t^3 - (9*t^2)/2 + 6*t - 2;
        v = 3*t^2 - 9*t + 6;
        a = 6*t - 9;

    end
end