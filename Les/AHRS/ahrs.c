/**
 ******************************************************************************
 * @addtogroup AHRS AHRS Control
 * @brief The AHRS Modules perform
 *
 * @{
 * @addtogroup AHRS_Main
 * @brief Main function which does the hardware dependent stuff
 * @{
 *
 *
 * @file       ahrs.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      INSGPS Test Program
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* OpenPilot Includes */
#include "ahrs.h"
#include "ahrs_adc.h"
#include "ahrs_timer.h"
//#include "pios_opahrs_proto.h"
#include "insgps.h"
#include "CoordinateConversions.h"
#include "ahrs_spi_comm.h"


// For debugging the raw sensors
//#define DUMP_RAW
//#define DUMP_FRIENDLY
//#define DUMP_EKF

#ifdef DUMP_EKF
#define NUMX 13			// number of states, X is the state vector
#define NUMW 9			// number of plant noise inputs, w is disturbance noise vector
#define NUMV 10			// number of measurements, v is the measurement noise vector
#define NUMU 6			// number of deterministic inputs, U is the input vector
extern float F[NUMX][NUMX], G[NUMX][NUMW], H[NUMV][NUMX];	// linearized system matrices
extern float P[NUMX][NUMX], X[NUMX];	// covariance matrix and state vector
extern float Q[NUMW], R[NUMV];	// input noise and measurement noise variances
extern float K[NUMX][NUMV];	// feedback gain matrix
#endif

/**
 * @addtogroup AHRS_Definitions
 * @{
 */
// Currently analog acquistion hard coded at 480 Hz
#define ADC_RATE          (4*480)
#define EKF_RATE          (ADC_RATE / adc_oversampling)
#define VDD            3.3	/* supply voltage for ADC */
#define FULL_RANGE     4096	/* 12 bit ADC */
#define ACCEL_RANGE    2	/* adjustable by FS input */
#define ACCEL_GRAVITY  9.81	/* m s^-1 */
#define ACCEL_SENSITIVITY    ( VDD / 5 )
#define ACCEL_SCALE    ( (VDD / FULL_RANGE) / ACCEL_SENSITIVITY * 2 / ACCEL_RANGE * ACCEL_GRAVITY )
#define ACCEL_OFFSET  -2048

#define GYRO_SENSITIVITY   ( 2.0 / 1000 )	/* 2 mV / (deg s^-1) */
#define RAD_PER_DEGREE     ( M_PI / 180 )
#define GYRO_SCALE         ( (VDD / FULL_RANGE) / GYRO_SENSITIVITY * RAD_PER_DEGREE )
#define GYRO_OFFSET       -1675	/* From data sheet, zero accel output is 1.35 v */

#define MAX_IDLE_COUNT          65e3
/**
 * @}
 */

/**
 * @addtogroup AHRS_Local Local Variables
 * @{
 */
struct mag_sensor {
	uint8_t id[4];
	uint8_t updated;
	struct {
		int16_t axis[3];
	} raw;
};

struct accel_sensor {
	struct {
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} filtered;
};

struct gyro_sensor {
	struct {
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} raw;
	struct {
		float x;
		float y;
		float z;
	} filtered;
	struct {
		uint16_t xy;
		uint16_t z;
	} temp;
};

struct attitude_solution {
	struct {
		float q1;
		float q2;
		float q3;
		float q4;
	} quaternion;
};

struct mag_sensor mag_data;
volatile struct accel_sensor accel_data;
volatile struct gyro_sensor gyro_data;

/**
 * @}
 */

/* Function Prototypes */
void downsample_data( void );
void calibrate_sensors( void );
void converge_insgps();
void calibration_callback( AhrsObjHandle obj );
void gps_callback( AhrsObjHandle obj );

volatile uint32_t last_counter_idle_start = 0;
volatile uint32_t last_counter_idle_end = 0;
volatile uint32_t idle_counts;
volatile uint32_t running_counts;
uint32_t counter_val;


/**
 * @addtogroup AHRS_Global_Data AHRS Global Data
 * @{
 * Public data.  Used by both EKF and the sender
 */

//!Raw attitude data.
//slightly ugly to do this with a global but it is fast and solves locking issues
AttitudeRawData attitude_raw;

//!GPS update flag
bool gps_updated = false;

//! Filter coefficients used in decimation.  Limited order so filter can't run between samples
int16_t fir_coeffs[50];

//! The oversampling rate, ekf is 2k / this
static uint8_t adc_oversampling = 1;
/**
 * @}
 */

/**
 * @brief AHRS Main function
 */

#define TEST_COMMS

int main()
{
	float gyro[3], accel[3], mag[3];
	float vel[3] = { 0, 0, 0 };
	/* Normaly we get/set UAVObjects but this one only needs to be set.
	We will never expect to get this from another module*/
	AttitudeActualData attitude_actual;
	AHRSSettingsData ahrs_settings;

	/* Brings up System using CMSIS functions, enables the LEDs. */
	PIOS_SYS_Init();

	/* Delay system */
	PIOS_DELAY_Init();

	/* Communication system */
	PIOS_COM_Init();

	/* ADC system */
	AHRS_ADC_Config( adc_oversampling );

	/* Setup the Accelerometer FS (Full-Scale) GPIO */
	PIOS_GPIO_Enable( 0 );
	SET_ACCEL_2G;
#if defined(PIOS_INCLUDE_HMC5843) && defined(PIOS_INCLUDE_I2C)
	/* Magnetic sensor system */
	PIOS_I2C_Init();
	PIOS_HMC5843_Init();

	// Get 3 ID bytes
	strcpy(( char * )mag_data.id, "ZZZ" );
	PIOS_HMC5843_ReadID( mag_data.id );
#endif

	/* SPI link to master */
//	PIOS_SPI_Init();
	AhrsInitComms();
	AHRSCalibrationConnectCallback( calibration_callback );
	GPSPositionConnectCallback( gps_callback );

	ahrs_state = AHRS_IDLE;

	while( !AhrsLinkReady() ) {
		AhrsPoll();
		while( ahrs_state != AHRS_DATA_READY ) ;
		ahrs_state = AHRS_PROCESSING;
		downsample_data();
		ahrs_state = AHRS_IDLE;
		if(( total_conversion_blocks % 50 ) == 0 )
			PIOS_LED_Toggle( LED1 );
	}


	AHRSSettingsGet(&ahrs_settings);


	/* Use simple averaging filter for now */
	for( int i = 0; i < adc_oversampling; i++ )
		fir_coeffs[i] = 1;
	fir_coeffs[adc_oversampling] = adc_oversampling;

	if( ahrs_settings.Algorithm ==  AHRSSETTINGS_ALGORITHM_INSGPS) {
		// compute a data point and initialize INS
		downsample_data();
		converge_insgps();
	}


#ifdef DUMP_RAW
	int previous_conversion;
	while( 1 ) {
		AhrsPoll();
		int result;
		uint8_t framing[16] = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
			15
		};
		while( ahrs_state != AHRS_DATA_READY ) ;
		ahrs_state = AHRS_PROCESSING;

		if( total_conversion_blocks != previous_conversion + 1 )
			PIOS_LED_On( LED1 );	// not keeping up
		else
			PIOS_LED_Off( LED1 );
		previous_conversion = total_conversion_blocks;

		downsample_data();
		ahrs_state = AHRS_IDLE;;

		// Dump raw buffer
		result = PIOS_COM_SendBuffer( PIOS_COM_AUX, &framing[0], 16 );	// framing header
		result += PIOS_COM_SendBuffer( PIOS_COM_AUX, ( uint8_t * ) & total_conversion_blocks, sizeof( total_conversion_blocks ) );	// dump block number
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX,
								 ( uint8_t * ) & valid_data_buffer[0],
								 ADC_OVERSAMPLE *
								 ADC_CONTINUOUS_CHANNELS *
								 sizeof( valid_data_buffer[0] ) );
		if( result == 0 )
			PIOS_LED_Off( LED1 );
		else {
			PIOS_LED_On( LED1 );
		}
	}
#endif

	timer_start();

	/******************* Main EKF loop ****************************/
	while( 1 ) {
		AhrsPoll();
		AHRSCalibrationData calibration;
		AHRSCalibrationGet( &calibration );
		BaroAltitudeData baro_altitude;
		BaroAltitudeGet( &baro_altitude );
		GPSPositionData gps_position;
		GPSPositionGet( &gps_position );
		AHRSSettingsGet(&ahrs_settings);

		// Alive signal
		if(( total_conversion_blocks % 100 ) == 0 )
			PIOS_LED_Toggle( LED1 );

#if defined(PIOS_INCLUDE_HMC5843) && defined(PIOS_INCLUDE_I2C)
		// Get magnetic readings
		if( PIOS_HMC5843_NewDataAvailable() ) {
			PIOS_HMC5843_ReadMag( mag_data.raw.axis );
			mag_data.updated = 1;
		}
		attitude_raw.magnetometers[0] = mag_data.raw.axis[0];
		attitude_raw.magnetometers[2] = mag_data.raw.axis[1];
		attitude_raw.magnetometers[2] = mag_data.raw.axis[2];

#endif
		// Delay for valid data

		counter_val = timer_count();
		running_counts = counter_val - last_counter_idle_end;
		last_counter_idle_start = counter_val;

		while( ahrs_state != AHRS_DATA_READY ) ;

		counter_val = timer_count();
		idle_counts = counter_val - last_counter_idle_start;
		last_counter_idle_end = counter_val;

		ahrs_state = AHRS_PROCESSING;

		downsample_data();

		/***************** SEND BACK SOME RAW DATA ************************/
		// Hacky - grab one sample from buffer to populate this.  Need to send back
		// all raw data if it's happening
		accel_data.raw.x = valid_data_buffer[0];
		accel_data.raw.y = valid_data_buffer[2];
		accel_data.raw.z = valid_data_buffer[4];

		gyro_data.raw.x = valid_data_buffer[1];
		gyro_data.raw.y = valid_data_buffer[3];
		gyro_data.raw.z = valid_data_buffer[5];

		gyro_data.temp.xy = valid_data_buffer[6];
		gyro_data.temp.z = valid_data_buffer[7];

		if( ahrs_settings.Algorithm ==  AHRSSETTINGS_ALGORITHM_INSGPS) {
			/******************** INS ALGORITHM **************************/

			// format data for INS algo
			gyro[0] = gyro_data.filtered.x;
			gyro[1] = gyro_data.filtered.y;
			gyro[2] = gyro_data.filtered.z;
			accel[0] = accel_data.filtered.x,
					   accel[1] = accel_data.filtered.y,
								  accel[2] = accel_data.filtered.z,
											 // Note: The magnetometer driver returns registers X,Y,Z from the chip which are
											 // (left, backward, up).  Remapping to (forward, right, down).
											 mag[0] = -( mag_data.raw.axis[1] - calibration.mag_bias[1] );
			mag[1] = -( mag_data.raw.axis[0] - calibration.mag_bias[0] );
			mag[2] = -( mag_data.raw.axis[2] - calibration.mag_bias[2] );

			INSStatePrediction( gyro, accel,
								1 / ( float )EKF_RATE );
			INSCovariancePrediction( 1 / ( float )EKF_RATE );

			if( gps_updated && gps_position.Status == GPSPOSITION_STATUS_FIX3D ) {
				// Compute velocity from Heading and groundspeed
				vel[0] =
					gps_position.Groundspeed *
					cos( gps_position.Heading * M_PI / 180 );
				vel[1] =
					gps_position.Groundspeed *
					sin( gps_position.Heading * M_PI / 180 );

				// Completely unprincipled way to make the position variance
				// increase as data quality decreases but keep it bounded
				// Variance becomes 40 m^2 and 40 (m/s)^2 when no gps
				INSSetPosVelVar( 0.004 );

				HomeLocationData home;
				HomeLocationGet( &home );
				float ned[3];
				double lla[3] = {( double ) gps_position.Latitude / 1e7, ( double ) gps_position.Longitude / 1e7, ( double )( gps_position.GeoidSeparation + gps_position.Altitude )};
				// convert from cm back to meters
				double ecef[3] = {( double )( home.ECEF[0] / 100 ), ( double )( home.ECEF[1] / 100 ), ( double )( home.ECEF[2] / 100 )};
				LLA2Base( lla, ecef, ( float( * )[3] ) home.RNE, ned );

				if( gps_updated ) { //FIXME: Is this correct?
					//TOOD: add check for altitude updates
					FullCorrection( mag, ned,
									vel,
									baro_altitude.Altitude );
					gps_updated = false;
				} else {
					GpsBaroCorrection( ned,
									   vel,
									   baro_altitude.Altitude );
				}

				gps_updated = false;
				mag_data.updated = 0;
			} else if( gps_position.Status == GPSPOSITION_STATUS_FIX3D
					   && mag_data.updated == 1 ) {
				MagCorrection( mag );	// only trust mags if outdoors
				mag_data.updated = 0;
			} else {
				// Indoors, update with zero position and velocity and high covariance
				INSSetPosVelVar( 0.1 );
				vel[0] = 0;
				vel[1] = 0;
				vel[2] = 0;

				VelBaroCorrection( vel,
								   baro_altitude.Altitude );
//                MagVelBaroCorrection(mag,vel,altitude_data.altitude);  // only trust mags if outdoors
			}

			attitude_actual.q1 = Nav.q[0];
			attitude_actual.q2 = Nav.q[1];
			attitude_actual.q3 = Nav.q[2];
			attitude_actual.q4 = Nav.q[3];
		} else if( ahrs_settings.Algorithm ==  AHRSSETTINGS_ALGORITHM_SIMPLE ) {
			float q[4];
			float rpy[3];
			/***************** SIMPLE ATTITUDE FROM NORTH AND ACCEL ************/
			/* Very simple computation of the heading and attitude from accel. */
			rpy[2] =
				atan2(( mag_data.raw.axis[0] ),
					  ( -1 * mag_data.raw.axis[1] ) ) * 180 /
				M_PI;
			rpy[1] =
				atan2( accel_data.filtered.x,
					   accel_data.filtered.z ) * 180 / M_PI;
			rpy[0] =
				atan2( accel_data.filtered.y,
					   accel_data.filtered.z ) * 180 / M_PI;

			RPY2Quaternion( rpy, q );
			attitude_actual.q1 = q[0];
			attitude_actual.q2 = q[1];
			attitude_actual.q3 = q[2];
			attitude_actual.q4 = q[3];
		}

		ahrs_state = AHRS_IDLE;

#ifdef DUMP_FRIENDLY
		PIOS_COM_SendFormattedStringNonBlocking( PIOS_COM_AUX, "b: %d\r\n",
				total_conversion_blocks );
		PIOS_COM_SendFormattedStringNonBlocking( PIOS_COM_AUX, "a: %d %d %d\r\n",
				( int16_t )( accel_data.filtered.x * 1000 ),
				( int16_t )( accel_data.filtered.y * 1000 ),
				( int16_t )( accel_data.filtered.z * 1000 ) );
		PIOS_COM_SendFormattedStringNonBlocking( PIOS_COM_AUX, "g: %d %d %d\r\n",
				( int16_t )( gyro_data.filtered.x * 1000 ),
				( int16_t )( gyro_data.filtered.y * 1000 ),
				( int16_t )( gyro_data.filtered.z * 1000 ) );
		PIOS_COM_SendFormattedStringNonBlocking( PIOS_COM_AUX, "m: %d %d %d\r\n",
				mag_data.raw.axis[0],
				mag_data.raw.axis[1],
				mag_data.raw.axis[2] );
		PIOS_COM_SendFormattedStringNonBlocking( PIOS_COM_AUX,
				"q: %d %d %d %d\r\n",
				( int16_t )( Nav.q[0] * 1000 ),
				( int16_t )( Nav.q[1] * 1000 ),
				( int16_t )( Nav.q[2] * 1000 ),
				( int16_t )( Nav.q[3] * 1000 ) );
#endif
#ifdef DUMP_EKF
		uint8_t framing[16] = {
			15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
			0
		};
		extern float F[NUMX][NUMX], G[NUMX][NUMW], H[NUMV][NUMX];	// linearized system matrices
		extern float P[NUMX][NUMX], X[NUMX];	// covariance matrix and state vector
		extern float Q[NUMW], R[NUMV];	// input noise and measurement noise variances
		extern float K[NUMX][NUMV];	// feedback gain matrix

		// Dump raw buffer
		int8_t result;
		result = PIOS_COM_SendBuffer( PIOS_COM_AUX, &framing[0], 16 );	// framing header
		result += PIOS_COM_SendBuffer( PIOS_COM_AUX, ( uint8_t * ) & total_conversion_blocks, sizeof( total_conversion_blocks ) );	// dump block number
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX,
								 ( uint8_t * ) & mag_data,
								 sizeof( mag_data ) );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX,
								 ( uint8_t * ) & gps_data,
								 sizeof( gps_data ) );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX,
								 ( uint8_t * ) & accel_data,
								 sizeof( accel_data ) );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX,
								 ( uint8_t * ) & gyro_data,
								 sizeof( gyro_data ) );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX, ( uint8_t * ) & Q,
								 sizeof( float ) * NUMX * NUMX );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX, ( uint8_t * ) & K,
								 sizeof( float ) * NUMX * NUMV );
		result +=
			PIOS_COM_SendBuffer( PIOS_COM_AUX, ( uint8_t * ) & X,
								 sizeof( float ) * NUMX * NUMX );

		if( result == 0 )
			PIOS_LED_Off( LED1 );
		else {
			PIOS_LED_On( LED1 );
		}
#endif
		AttitudeActualSet( &attitude_actual );

		/*FIXME: This is dangerous. There is no locking for UAVObjects
		so it could stomp all over the airspeed/climb rate etc.
		This used to be done in the OP module which was bad.
		Having ~4ms latency for the round trip makes it worse here.
		*/
		PositionActualData pos;
		PositionActualGet( &pos );
		for( int ct = 0; ct < 3; ct++ ) {
			pos.NED[ct] = Nav.Pos[ct];
			pos.Vel[ct] = Nav.Vel[ct];
		}
		PositionActualSet( &pos );

		static bool was_calibration = false;
		AhrsStatusData status;
		AhrsStatusGet( &status );
		if( was_calibration != status.CalibrationSet ) {
			was_calibration = status.CalibrationSet;
			if( status.CalibrationSet ) {
				calibrate_sensors();
				AhrsStatusGet( &status );
				status.CalibrationSet = true;
			}
		}
		status.CPULoad = (( float )running_counts /
						  ( float )( idle_counts + running_counts ) ) * 100;

		status.IdleTimePerCyle = idle_counts / ( TIMER_RATE / 10000 );
		status.RunningTimePerCyle = running_counts / ( TIMER_RATE / 10000 );
		status.DroppedUpdates = ekf_too_slow;
		AhrsStatusSet( &status );

	}

	return 0;
}

/**
 * @brief Downsample the analog data
 * @return none
 *
 * Tried to make as much of the filtering fixed point when possible.  Need to account
 * for offset for each sample before the multiplication if filter not a boxcar.  Could
 * precompute fixed offset as sum[fir_coeffs[i]] * ACCEL_OFFSET.  Puts data into global
 * data structures @ref accel_data and @ref gyro_data.
 *
 * The accel_data values are converted into a coordinate system where X is forwards along
 * the fuselage, Y is along right the wing, and Z is down.
 */
void downsample_data()
{
	int32_t accel_raw[3], gyro_raw[3];
	uint16_t i;

	AHRSCalibrationData calibration;
	AHRSCalibrationGet( &calibration );

	// Get the Y data.  Third byte in.  Convert to m/s
	accel_raw[0] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		accel_raw[0] +=
			( valid_data_buffer[0 + i * PIOS_ADC_NUM_PINS] +
			  calibration.accel_bias[1] ) * fir_coeffs[i];
	accel_data.filtered.y =
		( float )accel_raw[0] / ( float )fir_coeffs[adc_oversampling] *
		calibration.accel_scale[1];

	// Get the X data which projects forward/backwards.  Fifth byte in.  Convert to m/s
	accel_raw[1] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		accel_raw[1] +=
			( valid_data_buffer[2 + i * PIOS_ADC_NUM_PINS] +
			  calibration.accel_bias[0] ) * fir_coeffs[i];
	accel_data.filtered.x =
		( float )accel_raw[1] / ( float )fir_coeffs[adc_oversampling] *
		calibration.accel_scale[0];

	// Get the Z data.  Third byte in.  Convert to m/s
	accel_raw[2] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		accel_raw[2] +=
			( valid_data_buffer[4 + i * PIOS_ADC_NUM_PINS] +
			  calibration.accel_bias[2] ) * fir_coeffs[i];
	accel_data.filtered.z =
		-( float )accel_raw[2] / ( float )fir_coeffs[adc_oversampling] *
		calibration.accel_scale[2];

	// Get the X gyro data.  Seventh byte in.  Convert to deg/s.
	gyro_raw[0] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		gyro_raw[0] +=
			( valid_data_buffer[1 + i * PIOS_ADC_NUM_PINS] +
			  calibration.gyro_bias[0] ) * fir_coeffs[i];
	gyro_data.filtered.x =
		( float )gyro_raw[0] / ( float )fir_coeffs[adc_oversampling] *
		calibration.gyro_scale[0];

	// Get the Y gyro data.  Second byte in.  Convert to deg/s.
	gyro_raw[1] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		gyro_raw[1] +=
			( valid_data_buffer[3 + i * PIOS_ADC_NUM_PINS] +
			  calibration.gyro_bias[1] ) * fir_coeffs[i];
	gyro_data.filtered.y =
		( float )gyro_raw[1] / ( float )fir_coeffs[adc_oversampling] *
		calibration.gyro_scale[1];

	// Get the Z gyro data.  Fifth byte in.  Convert to deg/s.
	gyro_raw[2] = 0;
	for( i = 0; i < adc_oversampling; i++ )
		gyro_raw[2] +=
			( valid_data_buffer[5 + i * PIOS_ADC_NUM_PINS] +
			  calibration.gyro_bias[2] ) * fir_coeffs[i];
	gyro_data.filtered.z =
		( float )gyro_raw[2] / ( float )fir_coeffs[adc_oversampling] *
		calibration.gyro_scale[2];


	attitude_raw.gyros_filtered[0] = ( float )( valid_data_buffer[1] + calibration.gyro_bias[0] ) * calibration.gyro_scale[0];
	attitude_raw.gyros_filtered[1] = ( float )( valid_data_buffer[3] + calibration.gyro_bias[1] ) * calibration.gyro_scale[1];
	attitude_raw.gyros_filtered[2] = ( float )( valid_data_buffer[5] + calibration.gyro_bias[2] ) * calibration.gyro_scale[2];

	/*
		attitude_raw.gyros_filtered[0] = gyro_data.filtered.x;
		attitude_raw.gyros_filtered[1] = gyro_data.filtered.y;
		attitude_raw.gyros_filtered[2] = gyro_data.filtered.z;

		attitude_raw.accels_filtered[0] = accel_data.filtered.x;
		attitude_raw.accels_filtered[1] = accel_data.filtered.y;
		attitude_raw.accels_filtered[2] = accel_data.filtered.z;*/
	AttitudeRawSet( &attitude_raw );
}

/**
 * @brief Assumes board is not moving computes biases and variances of sensors
 * @returns None
 *
 * All data is stored in global structures.  This function should be called from OP when
 * aircraft is in stable state and then the data stored to SD card.
 */
void calibrate_sensors()
{
	int i;
	int16_t mag_raw[3] = { 0, 0, 0 };
	// local biases for noise analysis
	float accel_bias[3], gyro_bias[3], mag_bias[3];
	AHRSCalibrationData calibration;
	AHRSCalibrationGet( &calibration );

	// run few loops to get mean
	gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0;
	accel_bias[0] = accel_bias[1] = accel_bias[2] = 0;
	mag_bias[0] = mag_bias[1] = mag_bias[2] = 0;
	for( i = 0; i < 50; i++ ) {
		while( ahrs_state != AHRS_DATA_READY ) ;
		ahrs_state = AHRS_PROCESSING;
		downsample_data();
		gyro_bias[0] += gyro_data.filtered.x;
		gyro_bias[1] += gyro_data.filtered.y;
		gyro_bias[2] += gyro_data.filtered.z;
		accel_bias[0] += accel_data.filtered.x;
		accel_bias[1] += accel_data.filtered.y;
		accel_bias[2] += accel_data.filtered.z;
#if defined(PIOS_INCLUDE_HMC5843) && defined(PIOS_INCLUDE_I2C)
		PIOS_HMC5843_ReadMag( mag_raw );
#endif
		mag_bias[0] += mag_raw[0];
		mag_bias[1] += mag_raw[1];
		mag_bias[2] += mag_raw[2];

		ahrs_state = AHRS_IDLE;
	}
	gyro_bias[0] /= i;
	gyro_bias[1] /= i;
	gyro_bias[2] /= i;
	accel_bias[0] /= i;
	accel_bias[1] /= i;
	accel_bias[2] /= i;
	mag_bias[0] /= i;
	mag_bias[1] /= i;
	mag_bias[2] /= i;

	// more iterations for variance
	calibration.accel_var[0] = calibration.accel_var[1] = calibration.accel_var[2] = 0;
	calibration.gyro_var[0] = calibration.gyro_var[1] = calibration.gyro_var[2] = 0;
	calibration.mag_var[0] = calibration.mag_var[1] = calibration.mag_var[2] = 0;
	for( i = 0; i < 500; i++ ) {
		while( ahrs_state != AHRS_DATA_READY ) ;
		ahrs_state = AHRS_PROCESSING;
		downsample_data();
		calibration.gyro_var[0] +=
			( gyro_data.filtered.x -
			  gyro_bias[0] ) * ( gyro_data.filtered.x - gyro_bias[0] );
		calibration.gyro_var[1] +=
			( gyro_data.filtered.y -
			  gyro_bias[1] ) * ( gyro_data.filtered.y - gyro_bias[1] );
		calibration.gyro_var[2] +=
			( gyro_data.filtered.z -
			  gyro_bias[2] ) * ( gyro_data.filtered.z - gyro_bias[2] );
		calibration.accel_var[0] +=
			( accel_data.filtered.x -
			  accel_bias[0] ) * ( accel_data.filtered.x -
								  accel_bias[0] );
		calibration.accel_var[1] +=
			( accel_data.filtered.y -
			  accel_bias[1] ) * ( accel_data.filtered.y -
								  accel_bias[1] );
		calibration.accel_var[2] +=
			( accel_data.filtered.z -
			  accel_bias[2] ) * ( accel_data.filtered.z -
								  accel_bias[2] );
#if defined(PIOS_INCLUDE_HMC5843) && defined(PIOS_INCLUDE_I2C)
		PIOS_HMC5843_ReadMag( mag_raw );
#endif
		calibration.mag_var[0] +=
			( mag_raw[0] - mag_bias[0] ) * ( mag_raw[0] -
											 mag_bias[0] );
		calibration.mag_var[1] +=
			( mag_raw[1] - mag_bias[1] ) * ( mag_raw[1] -
											 mag_bias[1] );
		calibration.mag_var[2] +=
			( mag_raw[2] - mag_bias[2] ) * ( mag_raw[2] -
											 mag_bias[2] );
		ahrs_state = AHRS_IDLE;
	}
	calibration.gyro_var[0] /= i;
	calibration.gyro_var[1] /= i;
	calibration.gyro_var[2] /= i;
	calibration.accel_var[0] /= i;
	calibration.accel_var[1] /= i;
	calibration.accel_var[2] /= i;
	calibration.mag_var[0] /= i;
	calibration.mag_var[1] /= i;
	calibration.mag_var[2] /= i;

	float mag_length2 =
		mag_bias[0] * mag_bias[0] + mag_bias[1] * mag_bias[1] +
		mag_bias[2] * mag_bias[2];
	calibration.mag_var[0] = calibration.mag_var[0] / mag_length2;
	calibration.mag_var[1] = calibration.mag_var[1] / mag_length2;
	calibration.mag_var[2] = calibration.mag_var[2] / mag_length2;
	AHRSCalibrationSet( &calibration );
	AHRSSettingsData settings;
	AHRSSettingsGet(&settings);
	if( settings.Algorithm ==  AHRSSETTINGS_ALGORITHM_INSGPS )
		converge_insgps();
}

/**
 * @brief Quickly initialize INS assuming stationary and gravity is down
 *
 * Currently this is done iteratively but I'm sure it can be directly computed
 * when I sit down and work it out
 */
void converge_insgps()
{
	AHRSCalibrationData calibration;
	AHRSCalibrationGet( &calibration );
	float pos[3] = { 0, 0, 0 }, vel[3] = {
		0, 0, 0
	}, BaroAlt = 0, mag[3], accel[3], temp_gyro[3] = {
		0, 0, 0
	};
	INSGPSInit();
	INSSetAccelVar( calibration.accel_var );
	INSSetGyroBias( temp_gyro );	// set this to zero - crude bias corrected from downsample_data
	INSSetGyroVar( calibration.gyro_var );
	INSSetMagVar( calibration.mag_var );

	float temp_var[3] = { 10, 10, 10 };
	INSSetGyroVar( temp_var );	// ignore gyro's

	accel[0] = accel_data.filtered.x;
	accel[1] = accel_data.filtered.y;
	accel[2] = accel_data.filtered.z;

	// Iteratively constrain pitch and roll while updating yaw to align magnetic axis.
	for( int i = 0; i < 50; i++ ) {
		// This should be done directly but I'm too dumb.
		float rpy[3];
		Quaternion2RPY( Nav.q, rpy );
		rpy[1] =
			-atan2( accel_data.filtered.x,
					accel_data.filtered.z ) * 180 / M_PI;
		rpy[0] =
			-atan2( accel_data.filtered.y,
					accel_data.filtered.z ) * 180 / M_PI;
		// Get magnetic readings
#if defined(PIOS_INCLUDE_HMC5843) && defined(PIOS_INCLUDE_I2C)
		PIOS_HMC5843_ReadMag( mag_data.raw.axis );
#endif
		mag[0] = -mag_data.raw.axis[1];
		mag[1] = -mag_data.raw.axis[0];
		mag[2] = -mag_data.raw.axis[2];

		RPY2Quaternion( rpy, Nav.q );
		INSStatePrediction( temp_gyro, accel, 1 / ( float )EKF_RATE );
		INSCovariancePrediction( 1 / ( float )EKF_RATE );
		FullCorrection( mag, pos, vel, BaroAlt );
	}

	INSSetGyroVar( calibration.gyro_var );

}


/**
 * @brief AHRS calibration callback
 *
 * Called when the OP board sets the calibration
 */
void calibration_callback( AhrsObjHandle obj )
{

	AHRSCalibrationData data;
	AHRSCalibrationGet( &data );
	INSSetAccelVar( data.accel_var );
	float gyro_bias_ins[3] = { 0, 0, 0 };
	INSSetGyroBias( gyro_bias_ins );	//gyro bias corrects in preprocessing
	INSSetGyroVar( data.gyro_var );
	INSSetMagVar( data.mag_var );
}

/**
 * @brief GPS position callback
 *
 * Called when the GPS position changes
 */
void gps_callback( AhrsObjHandle obj )
{
	gps_updated = true;
}


/**
 * @}
 */
