function q = quintic_trajectory_wudao(t, tstart, tend, qstart, qend, qdotstart, qdotend)
% Quintic interpolation.
% the curve's acceleration at both the start and end are zero

% qstart = a0 + a1*tstart + a2*tstart^2 + a3*tstart^3 + a4*tstart^4 + a5*tstart^5
% qend = a0 + a1*tend + a2*tend^2 + a3*tend^3 0+ a4*tend^4 + a5*tendt^5
% qdotstart = a1 + 2*a2*tstart + 3*a3*tstart^2 + 4*a4*tstart^3 + 5*a5*tstart^4
% qdotend = a1 + 2*a2*tend + 3*a3*tend^2 + 4*a4*tend^3 + 5*a5*tend^4
% astart = 2*a2+ 6*a3*tstart+ 12*a4*tstart^2 + 20*a5*tstart^3 = 0
% aend = 2*a2+ 6*a3*tend + 12*a4*tend^2 + 20*a5*tend^3 = 0

Q = [1 tstart (tstart)^2 (tstart)^3 (tstart)^4 (tstart)^5;1 tend (tend)^2 (tend)^3 (tend)^4 (tend)^5;
0 1 2*tstart 3*(tstart)^2 4*(tstart)^3 5*(tstart)^4 ; 0 1 2*tend 3*(tend)^2 4*(tend)^3 5*(tend)^4;
0 0 2 6*tstart 12*(tstart)^2 20*(tstart)^3; 0 0 2 6*tend 12*(tend)^2 20*(tend)^3];
% Q * [a0 a1 a2 a3 a4 a5]' = [qstart qend qdotstart qdotend astart aend]'
% accelerations are zero at the start and ending

A = Q\[qstart qend qdotstart qdotend 0 0]';
% solve system of linear equation A = [a0 a1 a2 a3 a4 a5]'

q = A(1) + A(2)*t + A(3)*t^2 + A(4)*t^3 + A(5)*t^4 + A(6)*t^5;
% Quintic polynomial q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
