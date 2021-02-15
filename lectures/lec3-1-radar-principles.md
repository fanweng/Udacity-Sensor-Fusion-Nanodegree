# Lecture 3-1 Radar Principles

Radar is very efficient with a low cost and small footprint. It is capable to determine the targets at long range with accurate velocity and spatial information. Additionally, its sensitivity in dark and poor weather conditions also helps to cover the domains where Lidar or camera may fail.

<img src="media/radar-sensor-on-av.png" width="800" height="400" />

- Operation
    * transmit electromagnetic waves, and receive reflected wave from targets
- Construction
    * **radar dome** or **radome**: enclosure that protects a radar antenna, and it is effectively transparent to radio waves
    * **radar PCB**: analog hardware that includes transceiver, antenna
    * **DSP PCB**: DSP unit
- Comparison with Lidar
    * pros: operates in the bad weathers, low-cost, accurate velocity detection
    * cons: lower resolution

<img src="media/radar-vs-lidar.png" width="800" height="350" />



### I. Signal Properties

- Wavelength
    * physical length from one point of a wave to the same point on the next wave
    * wavelength = speed_of_light / frequency
- Frequency
    * first derivative of the phase with respect to the time
    * automotive radar operates at W band (76GHz - 81GHz), a.k.a mmWave
- Bandwidth
    * difference between the highest and the lowest frequency components
- Amplitude
    * strength of the signal
    * automotive radar operates at max of 55dBm output power (316W)
- Phase
    * a particular point in time on the cycle of a waveform, measured as an angle in degrees

#### General equation of a wave

<img src="media/general-equation-a-wave.png" width="300" height="50" />



### II. FMCW

Frequency-Modulated Continuous Wave (FMCW) radar radiates continuous transmission power. Its signal frequency increases/decreases with time. Two most common waveform patterns for FMCW:
- sawtooth (upramps)
- triangular (both upramps and downramps)

**Chirp** is defined by its **slope**. The slope is given by its chirp frequency bandwidth (B, y-axis) and its chirp time (T, x-axis). The range resolution requirement decides the *B*, whereas the maximum velocity capability of a radar is determined by the chirp time *T*.

<img src="media/fmcw-slope.png" width="180" height="60" />

#### Antenna details

The antenna pattern below shows the strength of the relative field emitted by the antenna. The *beamwidth* determines the field of view for the radar sensor. A wider beamwidth will sense the target in other lanes. If the requirement is to just sense the targets in its own lane, the beamwidth needs to be small enough. Antenna *sidelobes* could generate false alarms and pick interference from undesired direction. Thus, it is critical to suppress the sidelobe strength level.

<img src="media/antenna-pattern.png" width="800" height="400" />



### III. Radar Cross Section

The size and ability of a target to reflect radar energy is defined by **radar cross-section (RCS)**. It depends on:

- target's physical geometry and exterior features
- direction of the illuminating radar
- radar transmitter's frequency
- target's material properties

The unit of RCS can be defined using *square meter* or *dB*.

<img src="media/rcs-units.png" width="200" height="30" />



### IV. Radar Range Equation

<img src="media/radar-range-equation.png" width="700" height="200" />

#### Signal-to-noise ratio

To successfully detect a target, the return signal strength needs to be larger than the noise level, i.e. signal-to-noise (SNR) level needs to be high. Generally, a 7 to 13 dB SNR ensures a successful detection in a road scenario.

#### Exercise: maximum range calculation

With the given parameters, it is possible to calculate the wavelength and max detection range for radar as below. ([8b60aee](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/8b60aeef0fd3cd16c9c3220203eee668dc8196f5))

```matlab
%Task 1: Calculate the wavelength
lambda = c / fc;
fprintf('wavelength = %.3f mm\n', lambda * 1000);

%Task 2: Measure the Maximum Range a Radar can see
R = nthroot((Ps * G^2 * lambda^2 * RCS / (Pe * (4 * pi)^3)), 4);
fprintf('Max detection range = %.3f m\n', R);
```

The computation results are:

```matlab
>> max_range_cal
wavelength = 3.896 mm
Max detection range = 218.871 m
```
