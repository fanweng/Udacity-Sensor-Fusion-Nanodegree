# Lecture 3-2 Range Doppler Estimation

In this lesson, we will learn the basics of range and velocity estimation using doppler and Fourier transformation techniques.

<img src="media/radar-resolutions.png" width="600" height="280" />

The distance, angle and velocity differences between two targets determine if the radar can resolve them.

- Range resolution
    * to distinguish between two targets that are close to each other
    * solely rely on the *chirp frequency bandwidth*: `dres = c / (2 * B)`
- Velocity resolution
    * if two targets travel in different velocities, radar still can resolve them even if they have the same range
    * rely on the *number of chirps*, higher the number, higher the resolution but also longer processing time
- Angle resolution
    * to separate two targets spatially when they have the same range and the same velocity



### I. Range Estimation

In general, for an FMCW radar system, the chirp (sweep) time should at least 5 to 6 times of the round trip time. We can use a factor of 5.5 for this lesson: `Ts = 5.5 * 2 * RadarMaxRange / c`.

By measuring the received frequency and subtracting it from the radar's ramping frequency, we can get the **beat frequency**, `Fb = Framping - Freceived`. Thus, range can be estimated by `R = c * Ts * Fb / (2 * Bsweep)`.

Here, `R` is range, `c` is speed of light, `Ts` is the chirp time, `Fb` is the beat frequency, `Bsweep` is the chirp bandwidth.

<img src="media/range-estimation.png" width="800" height="350" />

#### Exercise: range calculation

Using the MATLAB code ([36f29e1](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/36f29e1b2bba07c4db245c82ca68925658e30bec)), I calculated the ranges [0.0 m, 12.1 m, 143.0 m, 264.0 m] of four targets with measured beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz] respectively.

```matlab
>> range_cal
    0   12.1000  143.0000  264.0000
```