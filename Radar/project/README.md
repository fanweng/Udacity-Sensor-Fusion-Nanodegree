# Radar Target Generation and Detection Project

## Project Layout

- Configure the FMCW waveform based on the system requirements
- Define the range and velocity of target and simulated its displacement
- For the same simulation loop, process the  transmit and receive signal to determine the beat signal
- Perform Range FFT on the received signal to determine the Range
- Perform the CFAR processing on the output of 2nd FFT to display the target

<img src="media/radar-project-overview.png" width="700" height="400" />

#### System requirements

System requirements define the design of a radar.

| | |
|-|-|
| Frequency         | 77 GHz    |
| Range Resolution  | 1m        |
| Max Range         | 200 m     |
| Max Velocity      | 70 m/s    |
| Velocity Resolution | 3 m/s   |

The above requirements can yield radar specifications.

```matlab
% Speed of light
c = 3 * 10^8
% Sweep bandwidth
Bsweep = c / (2 * rangeResolution)
% Sweep time
Tchirp = 5.5 * 2 * maxRange / c
% Slope of chirp signal
Slope = Bsweep / Tchirp
```