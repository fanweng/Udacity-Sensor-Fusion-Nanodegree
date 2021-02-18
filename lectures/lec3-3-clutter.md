# Lecture 3-3 Cluter, CFAR, AoA

### I. Clutter

Radar signal would be also reflected from environment and unwanted objects (ground, buildings, trees, rain, etc.) besides from the targets. Those false signals are called **Clutter**.

Methods to remove the clutter:
- Remove signal having *zero* doppler velocity
    * because unwanted objects are usually stationary
    * downside: fail to detect stationary targets
- Fixed clutter thresholding
    * reject signal below a certain threshold value
    * downside
        + if set too high, it masks signals from valid targets
        + if set too low, too many false alarms will be generated
- Dynamic thresholding
    * vary the threshold level to reduce the false alarm rate
    * Example: Constant False Alarm Rate (CFAR)

### II. CFAR

Constant False Alarm Rate (CFAR) technique estimates the level of interference in radar range and doppler cells (*Training Cells*) on either or both sides of the *Cell Under Test* (CUT). Assume that the noise/interference is spatially or temporarily homogeneous, it will produce a constant false alarm rate, which is independent from noise or clutter level.

Multiple categories of CFAR:
- Cell Averaging CFAR (CA-CFAR)
- Ordered Statistics CFAR (OS-CFAR)
- Maximum Minimum Statistic (MAMIS CFAR)
- and others...

### III. CA-CFAR

As discussed in the [Lecture 3-2](./lec3-2-range-doppler-estimation.md), the FFT bins are generated on implementing range and doppler FFTs across a number of chirps. CFAR process includes the sliding of a window across the cells in FFT bins.

- Cell Under Test (CUT)
    * cell that is tested to detect the presence of the target by comparing the signal level against the noise threshold
- Training Cells
    * noise is estimated by averaging the noise under the training cells
- Guard Cells
    * cells next to CUT, preventing the target signal from leaking into the training cells
- Threshold Factor (Offset)
    * use an offset value to scale the noise threshold

<img src="media/ca-cfar.png" width="700" height="350" />

#### Exercise: 1D CFAR

Using the MATLAB code [c01621d](https://github.com/fanweng/Udacity-Sensor-Fusion-Nanodegree/commit/c01621d97646961b15de47536271ed53ed3eae15), I buried four target signals at Bin 100, 200, 300 and 700 with random noise. Then CA-CFAR was applied to extract four signals with proper configurations of number of training cells, guard cells and offset.

<img src="media/cfar-1d-result.png" width="700" height="500" />