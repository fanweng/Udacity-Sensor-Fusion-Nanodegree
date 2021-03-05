# Radar Target Generation and Detection Project

## Project Layout

- Configure the FMCW waveform based on the system requirements
- Define the range and velocity of target and simulated its displacement
- For the same simulation loop, process the  transmit and receive signal to determine the beat signal
- Perform Range FFT on the received signal to determine the Range
- Perform the CFAR processing on the output of 2nd FFT to display the target

<img src="media/radar-project-overview.png" width="700" height="400" />

### 1. System requirements

System requirements define the design of a radar.

| | |
|-|-|
| Frequency         | 77 GHz    |
| Range Resolution  | 1m        |
| Max Range         | 200 m     |
| Max Velocity      | 70 m/s    |
| Velocity Resolution | 3 m/s   |

The above requirements can yield radar specifications: `Bsweep`, `Tchirp`, and `Slope`.

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

### 2. Signal propagation

FMCW transmit and received signals are defined using the wave equations, where the alpha `a` is the `Slope`. The received signal is nothing but the time delayed version of the transmit signal.

On mixing these two signals, we get the beat signal by *element-by-element multiplication* of two signal matrices: `times(Tx, Rx)` or `Tx.*Rx`.

<img src="media/signal-propagation.png" width="400" height="100" />

### 3. FFT operation

By implementing the 2D FFT on the beat signal, we can extract both **Range** and **Doppler** information.

First FFT would be implemented on the mixed/beat signal, outputting a peak at the range of the vehicle. The second FFT will generate a range doppler map.

### 4. 2D CFAR

Use 2D CFAR to threshold the cells in the range doppler map generated in the last step.