Development branch to extend the calibration system.
1) Use full calibration of scale factor, bias, and orthogonality for the 
	accelerometer and magnetometer.
2) A low and high temperatures, add additional bias and scale factor 
	calibrations. 
2)a) Linearly interpolate for scale factor and bias as a function of 
	temperature.

3) For gyros, also calibrate the acceleration sensitivity of the gyro.

4) For the magnetometer, also calibrate the misalignment matrix between it and
 the accelerometer.

Treat the accelerometer's frame as the reference frame for calibration on the AHRS.

5) Calibrate alignment and scale factor for the gyros.

To be determined: How the scale factor changes with temperature.  The mfr datasheets
claim a high level of stability over temperature.  We'll have to collect data to
validate that claim.

Each calibration term is saved separately on the main board.  The AHRS will compute
an internal scaling matrix and offset vector from the saved parameters.