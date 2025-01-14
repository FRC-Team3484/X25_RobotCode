#ifndef SC_CONSTANTS_H
#define SC_CONSTANTS_H

#include "units/angle.h"
#include "units/time.h"
#include <tuple>

#define null nullptr

namespace SC
{
	/* General Constants */
	#define C_DISABLED_CHANNEL	-1

	#define C_SCAN_TIME         0.020_s // seconds
	#define C_SCAN_TIME_SEC		C_SCAN_TIME.value()

	const std::tuple<int, int> C_BLANK_IDS = std::make_tuple<int, int>(-1, -1);

	// Motor and Encoder values.
	const double C_TALONFX_CPR			= 2048.0;
	const double C_CTREMAG_CPR			= 4096.0;

	const double C_FALCON500_FREE_RPM	= 6380.0;
	const double C_CIM_FREE_RPM			= 5330.0;
	const double C_MINICIM_FREE_RPM		= 5840.0;
	const double C_775PRO_FREE_RPM		= 18730.0;
	const double C_BAG_FREE_RPM			= 13180.0;

	const double C_TALONFX_RPM_SF		= C_TALONFX_CPR / 600.0; // Multiply by (max RPM / GR) to get max output velocity value in RPM
	const double C_CTREMAG_RPM_SF		= C_CTREMAG_CPR / 600.0; // Multiply by (max RPM / GR) to get max output velocity value in RPM
}
#endif