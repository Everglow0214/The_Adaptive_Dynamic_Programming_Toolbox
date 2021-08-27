function [sys, u0, q] = sf2mf(fsym, gsym, xsym, usym,...
    u0sym, qsym)
% >> This function converts symbolic functions to MATLAB
% functions that can be used in ode45.
%
% >> [sys,u0,q] = sf2mf(fsym, gsym, xsym, usym, u0sym,
%       qsym)
% ======================== Input =========================
% fsym:  f(x) in the system dynamics
%        (symbolic representation)
% gsym:  g(x) in the system dynamics
%        (symbolic representation or just a matrix)
% xsym:  system state
%        (symbolic representation)
% usym:  control input
%        (symbolic representation)
% u0sym: initial admissible control
%        (symbolic representation)
% qsym:  q(x) in the performance index
%        (symbolic representation)
% ========================================================
% ======================== Output ========================
% sys:   system dynamics
%        (function handle)
% u0:    initial admissible control
%        (function handle)
% q:     penalty term in the performance index
%        (function handle)
% ========================================================

sys = matlabFunction(fsym+gsym*usym, 'Vars', {xsym,usym});
u0  = matlabFunction(u0sym, 'Vars', {xsym});
% e   = matlabFunction(esym, 'Vars', tsym);
q   = matlabFunction(qsym, 'Vars', {xsym});

end