%Constants
c = 3 * 10^8;

%Parameters
maxRange = 300;
rangeRes = 1;
factor = 5.5; %FMCW system usually has a sweep time 5 to 6 times of round trip time
Fb = [0 1.1*10^6 13*10^6 24*10^6]; %measured beat frequencies

%Task 1: Find the Bsweep of chirp for 1 m resolution
Bsweep = 0.5 * c / rangeRes;

%Task 2: Calculate the chirp time based on the Radar's Max Range
Ts = factor * 2 * maxRange / c;

%Task 3: define the frequency shifts
calculated_range = c * Ts * Fb / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);