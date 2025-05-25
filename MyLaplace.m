function y = MyLaplace(U,dt,a)
% Approximates the single-sided Laplace transform of the signal U,
% at a point a. The scalar a is a real and positive number.
%
% Inputs:
% U - a vector that holds samples of u(t):
% U = [u(0), u(dt) , u(2*dt) , ... , u(N*dt)]
% dt - the sampling time
% a - a real positive number: the point in which to evaluate the transform.
%
% Output:
% y - the approximated Laplace transform
%
% See more documentation about this function in the word file in the
% library.

eps1 = 1e-2; % If abs(I_tail/I_body)>eps1 the function will issue a warning.

N = length(U) - 1;
y = NaN;

if (~isreal(a))
    display('MyLaplace error: a must be a real number.');
    return;
elseif (a<=0)
    display('MyLaplace error: a<=0.');
    return;
elseif (N<2)
    display('MyLaplace error: not enough samples.');
    return;    
elseif (dt<=0)
    display('MyLaplace error: dt<=0.');
    return;    
end

U = reshape(U,1,N+1);
Uval = U(1:N);
Utag = diff(U)/dt;
ExpVals = exp(-a*(0:(N-1))*dt);
ca = (1-exp(-a*dt))/a;
cb = (1-exp(-a*dt)*(1+a*dt))/a^2;
Uavg = sum(U)/(N+1);

I_body = ca*sum(ExpVals.*Uval) + cb*sum(ExpVals.*Utag);
I_tail = (Uavg/a)*exp(-a*N*dt);

if (I_body ~= 0)
if (abs(I_tail/I_body)>eps1)
    display('MyLaplace warning: estimation may be inaccurate. Try to increase a, or pad the signal with zeros.\n');
end
end

y = I_body + I_tail;

end


