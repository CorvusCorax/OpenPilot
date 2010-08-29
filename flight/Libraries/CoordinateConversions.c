/**
 ******************************************************************************
 *
 * @file       CoordinateConversions.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      General conversions with different coordinate systems.
 *             - all angles in deg
 *             - distances in meters
 *             - altitude above WGS-84 elipsoid
 *
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

#include <math.h>
#include <stdint.h>
#include "CoordinateConversions.h"

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

// ****** convert Lat,Lon,Alt to ECEF  ************
void LLA2ECEF(double LLA[3], double ECEF[3]){
  const double a = 6378137.0;           // Equatorial Radius
  const double e = 8.1819190842622e-2;  // Eccentricity
  double sinLat, sinLon, cosLat, cosLon;
  double N;

	sinLat=sin(DEG2RAD*LLA[0]);
	sinLon=sin(DEG2RAD*LLA[1]);
	cosLat=cos(DEG2RAD*LLA[0]);
	cosLon=cos(DEG2RAD*LLA[1]);
    
	N = a / sqrt(1.0 - e*e*sinLat*sinLat);  //prime vertical radius of curvature

	ECEF[0] = (N+LLA[2])*cosLat*cosLon;
	ECEF[1] = (N+LLA[2])*cosLat*sinLon;
	ECEF[2] = ((1-e*e)*N + LLA[2]) * sinLat;
}
// ****** convert ECEF to Lat,Lon,Alt (ITERATIVE!) *********
uint16_t ECEF2LLA(double ECEF[3], double LLA[3])
{
  const double a = 6378137.0;           // Equatorial Radius
  const double e = 8.1819190842622e-2;  // Eccentricity
  double x=ECEF[0], y=ECEF[1], z=ECEF[2];
  double Lat, N, NplusH, delta, esLat;
  uint16_t iter;
#define MAX_ITER 100

	LLA[1] = RAD2DEG*atan2(y,x);
	N = a; 
	NplusH = N;       
	delta = 1;
	Lat = 1;
	iter=0;
	
	while (((delta > 1.0e-14)||(delta < -1.0e-14)) && (iter < MAX_ITER))
	{
		delta = Lat - atan(z / (sqrt(x*x + y*y)*(1-(N*e*e/NplusH))));
		Lat = Lat-delta;
		esLat = e*sin(Lat);
		N = a / sqrt(1 - esLat*esLat);
        NplusH = sqrt(x*x + y*y)/cos(Lat);
        iter += 1;
	}

	LLA[0] = RAD2DEG*Lat;
	LLA[2] = NplusH - N;

  return (iter < MAX_ITER);
}

// ****** find ECEF to NED rotation matrix ********
void RneFromLLA(double LLA[3], float Rne[3][3]){
  float sinLat, sinLon, cosLat, cosLon;

	sinLat=(float)sin(DEG2RAD*LLA[0]);
	sinLon=(float)sin(DEG2RAD*LLA[1]);
	cosLat=(float)cos(DEG2RAD*LLA[0]);
	cosLon=(float)cos(DEG2RAD*LLA[1]);
    
    Rne[0][0] = -sinLat*cosLon; Rne[0][1] = -sinLat*sinLon; Rne[0][2] = cosLat;
    Rne[1][0] = -sinLon;        Rne[1][1] = cosLon;         Rne[1][2] = 0;
    Rne[2][0] = -cosLat*cosLon; Rne[2][1] = -cosLat*sinLon; Rne[2][2] = -sinLat;
}

// ****** find roll, pitch, yaw from quaternion ********
void Quaternion2RPY(float q[4], float rpy[3]){
  float R13, R11, R12, R23, R33;
  float q0s=q[0]*q[0];
  float q1s=q[1]*q[1];
  float q2s=q[2]*q[2];
  float q3s=q[3]*q[3];

	R13 = 2*(q[1]*q[3]-q[0]*q[2]);
	R11 = q0s+q1s-q2s-q3s;
	R12 = 2*(q[1]*q[2]+q[0]*q[3]);
    R23 = 2*(q[2]*q[3]+q[0]*q[1]);
    R33 = q0s-q1s-q2s+q3s;

    rpy[1]=RAD2DEG*asinf(-R13);    // pitch always between -pi/2 to pi/2
    rpy[2]=RAD2DEG*atan2f(R12,R11);
    rpy[0]=RAD2DEG*atan2f(R23,R33);
}

// ****** find quaternion from roll, pitch, yaw ********
void RPY2Quaternion(float rpy[3], float q[4]){
  float phi, theta, psi;
  float cphi, sphi, ctheta, stheta, cpsi, spsi;

  phi=DEG2RAD*rpy[0]/2; theta=DEG2RAD*rpy[1]/2; psi=DEG2RAD*rpy[2]/2;
  cphi=cosf(phi); sphi=sinf(phi);
  ctheta=cosf(theta); stheta=sinf(theta);
  cpsi=cosf(psi); spsi=sinf(psi);
  
  q[0] = cphi*ctheta*cpsi + sphi*stheta*spsi;
  q[1] = sphi*ctheta*cpsi - cphi*stheta*spsi;
  q[2] = cphi*stheta*cpsi + sphi*ctheta*spsi;
  q[3] = cphi*ctheta*spsi - sphi*stheta*cpsi;
  
  if (q[0] < 0){    // q0 always positive for uniqueness
	  q[0]=-q[0];
	  q[1]=-q[1];
	  q[2]=-q[2];
	  q[3]=-q[3];
  }
}

//** Find Rbe, that rotates a vector from earth fixed to body frame, from quaternion **
void Quaternion2R(float q[4], float Rbe[3][3]){

  float q0s=q[0]*q[0], q1s=q[1]*q[1], q2s=q[2]*q[2], q3s=q[3]*q[3];

  Rbe[0][0]=q0s+q1s-q2s-q3s;
  Rbe[0][1]=2*(q[1]*q[2]+q[0]*q[3]);
  Rbe[0][2]=2*(q[1]*q[3]-q[0]*q[2]);
  Rbe[1][0]=2*(q[1]*q[2]-q[0]*q[3]);
  Rbe[1][1]=q0s-q1s+q2s-q3s;
  Rbe[1][2]=2*(q[2]*q[3]+q[0]*q[1]);
  Rbe[2][0]=2*(q[1]*q[3]+q[0]*q[2]);
  Rbe[2][1]=2*(q[2]*q[3]-q[0]*q[1]);
  Rbe[2][2]=q0s-q1s-q2s+q3s;
}

// ****** Express LLA in a local NED Base Frame ********
void LLA2Base(double LLA[3], double BaseECEF[3], float Rne[3][3], float NED[3]){
  double ECEF[3];
  float diff[3];

	LLA2ECEF(LLA,ECEF);

	diff[0]=(float)(ECEF[0]-BaseECEF[0]);
    diff[1]=(float)(ECEF[1]-BaseECEF[1]);
    diff[2]=(float)(ECEF[2]-BaseECEF[2]);

	NED[0]= Rne[0][0]*diff[0]+Rne[0][1]*diff[1]+Rne[0][2]*diff[2];
	NED[1]= Rne[1][0]*diff[0]+Rne[1][1]*diff[1]+Rne[1][2]*diff[2];
	NED[2]= Rne[2][0]*diff[0]+Rne[2][1]*diff[1]+Rne[2][2]*diff[2];
}

// ****** Express ECEF in a local NED Base Frame ********
void ECEF2Base(double ECEF[3], double BaseECEF[3], float Rne[3][3], float NED[3]){
  float diff[3];

	diff[0]=(float)(ECEF[0]-BaseECEF[0]);
    diff[1]=(float)(ECEF[1]-BaseECEF[1]);
    diff[2]=(float)(ECEF[2]-BaseECEF[2]);

	NED[0]= Rne[0][0]*diff[0]+Rne[0][1]*diff[1]+Rne[0][2]*diff[2];
	NED[1]= Rne[1][0]*diff[0]+Rne[1][1]*diff[1]+Rne[1][2]*diff[2];
	NED[2]= Rne[2][0]*diff[0]+Rne[2][1]*diff[1]+Rne[2][2]*diff[2];
}
