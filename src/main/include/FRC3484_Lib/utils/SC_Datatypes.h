#ifndef SC_DATATYPES_H
#define SC_DATATYPES_H


//#include "TC3_Syntax.h"
#include <tuple>
#include <algorithm>
#include <stdexcept>
#include <frc/PneumaticsModuleType.h>
#include <frc/geometry/Transform3d.h>

#include <units/math.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/controller/ArmFeedforward.h>

//using Acceleration = units::compound_unit<units::radians_per_second, units::inverse<units::seconds>>

namespace SC
{
	typedef struct {double Top_Power; units::revolutions_per_minute_t Top_Speed; double Bottom_Power; units::revolutions_per_minute_t Bottom_Speed;} SC_LauncherSpeed;

	typedef struct {double Kp; double Ki; double Kd; double Kf;} SC_PIDConstants;

	typedef struct {int CtrlID; frc::PneumaticsModuleType CtrlType; int Channel;} SC_Solenoid;
	typedef struct {int CtrlID; frc::PneumaticsModuleType CtrlType; int Fwd_Channel; int Rev_Channel;} SC_DoubleSolenoid;

	// Swerve Configurations
	typedef struct {
		units::ampere_t Drive_Current_Threshold = 60_A;
		units::second_t Drive_Current_Time = 0.1_s;

		units::ampere_t Steer_Current_Threshold = 40_A;
		units::second_t Steer_Current_Time = 0.1_s;

		bool Steer_Motor_Reversed = true;
		bool Encoder_Reversed = false;
		bool Current_Limit_Enable = true;
		units::ampere_t Current_Limit_Drive = 35_A;
		units::ampere_t Current_Limit_Steer = 25_A;

	} SC_SwerveCurrents;


	using kv_unit = units::compound_unit<units::volts, units::inverse<units::meters_per_second>>;
	using ka_unit = units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>;
	
	typedef struct {
		double Kp;
		double Ki;
		double Kd;
		units::unit_t<kv_unit> V;
		units::unit_t<ka_unit> A;
		units::volt_t S;
	} SC_SwervePID;
	
	using kav_unit = units::compound_unit<units::volts, units::inverse<units::rad_per_s>>;
	using kaa_unit = units::compound_unit<units::volts, units::inverse<units::rad_per_s_sq>>;
	typedef struct {
		units::volt_t G;
		units::volt_t S;
		units::unit_t<kav_unit> V;
		units::unit_t<kaa_unit> A;
	} SC_AngularFeedForward;



	typedef struct {
		units::volt_t G;
		units::volt_t S;
		units::unit_t<kv_unit> V;
		units::unit_t<ka_unit> A;
	} SC_LinearFeedForward;

	typedef struct {
		int CAN_ID;
		int SteerMotorPort;
		int EncoderPort;
		units::degree_t EncoderOffset;
	} SC_SwerveConfigs;


	// Vision Configurations
	typedef struct {
		std::string_view Camera_Name;
		frc::Transform3d Camera_Position;
		bool Camera_Enabled;
	} SC_CameraConfig;

	typedef struct {
		const frc::Pose2d Vision_Measurement;
		units::second_t Timestamp;
		const Eigen::Vector3d Standard_Deviation;
	} SC_CameraResults;

	// Amp Configurations
	typedef struct {
		double Motor_Current_Threshold = 50;
		double Motor_Current_Time = 0.1;
		bool Motor_Motor_Reversed = true;
		bool Encoder_Reversed = false;
		bool Current_Limit_Enable = true;
		double Current_Limit_Motor = 20;

	} SC_MotorCurrents;

	typedef struct {
		int CAN_ID;
		int SteerMotorPort;
		int EncoderPort;
		double EncoderOffset;
	} SC_MotorConfigs;

	// PID loop Integral Anti-Windup calculation modes
	enum SC_PID_AW_MODE { OFF, /*
						BACK_CALCULATION, 
						CONDITIONAL_INTEGRAL, */
						INTEGRAL_LIMIT };

	typedef struct {
		double P;   // Current value of the P term 
		double I;   // Current value of the I term
		double D;   // Current value of the D term
		double E;   // Current error value
		double Elast; // Last error value
		SC_PID_AW_MODE AntiWindup_Mode; // Configured Anti-windup method
		double Kw;  // Anti-Windup Coeff value
		bool enabled; // Enabled status of PID Controller
		bool reversed; // Reversed status of PID Controller
		double SP; // current setpoint
		double PV;
		double CV;
	} SC_PIDStatus;

	template<class T>
	struct SC_Range 
	{ 
		T Val_min; 
		T Val_max; 

		SC_Range& operator=(SC_Range& rhs)
		{
			this->Val_max = std::max(rhs.Val_max, rhs.Val_min);
			this->Val_min = std::min(rhs.Val_max, rhs.Val_min);

			return *this;
		}

		SC_Range& operator()(T val1, T val2)
		{
			this->Val_max = std::max(val1, val2);
			this->Val_min = std::min(val1, val2);

			return *this;
		}

		SC_Range& operator=(std::tuple<T, T> rhs)
		{
			this->Val_max = std::max(std::get<0>(rhs), std::get<1>(rhs));
			this->Val_min = std::min(std::get<0>(rhs), std::get<1>(rhs));

			return *this;
		}
	};

	template<class T>
	class SC_Array
	{
	public:
		SC_Array(uint size)
		{
			values = std::malloc(sizeof(T) * size);
			len = size;
		}

		SC_Array(uint size, T &initial)
		{
			values = std::malloc(sizeof(T) * size);
			len = size;
		}

		~SC_Array()
		{
			delete values;
			len = 0;
		}

		template<typename T2>
		SC_Array<T> operator+=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] += static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator-=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] -= static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator*=(T2 other)
		{
			for(uint i; i < len; i++) { values[i] *= static_cast<T>(other); }
			return this;
		}

		template<typename T2>
		SC_Array<T> operator/=(T2 val)
		{
			if(val = 0)
			{
				throw std::exception("Divide by zero error!");
				return this;
			}
			else
			{
				for(uint i = 0; i < this->len; i++) { this->values[i] /= static_cast<T>(val); }
				return this;
			}
		}

		SC_Array<T> operator[](int i)
		{
			if(std::abs(i) > this->len)
			{
				throw std::out_of_range("Index out of range! (SC_Array)");
				return this;
			}

			// Index from the back of the array, else normal indexing.
			if(i < 0) { return this->values[this-len - std::abs(i)]; }
			else { return this->values[i]; }
		}


	private:
		T *values;
		uint len;

	};

	enum DriveMode { DEFAULT, TANK, DIFFERENTIAL, MECANUM };
	enum SC_Wheel { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT, LEFT_WHEEL, RIGHT_WHEEL };

	/**
	 * Alpha-Beta Filter templated against the units library.
	 */
	template<class T>
	class SC_ABFilterU
	{
	public:
		SC_ABFilterU(units::time::second_t FilterTime, units::time::second_t ScanTime)
		{
			this->_tau = FilterTime.value();
			this->_scanT = ScanTime.value();
		}

		~SC_ABFilterU()
		{
			;
		}

		T Filter(T PV)
		{
			T pvf = PV; // Filtered input value

			if(this->_tau != this->_scanT)
			{
				this->_beta = this->_scanT / (this->_tau + this->_scanT);
				this->_alpha = 1 - this->_beta;

				pvf = (this->_alpha * PV_out) + (this->_beta * PV);

				PV_out = units::math::abs(pvf).value() < 1E-9 ? units::make_unit<T>(0.0) : pvf;

				return PV_out;

			}

			return pvf;
		}
	private:
		double _tau, _scanT;
		double _alpha, _beta;
		T PV_out;

	};

	/**
	 * Alpha-Beta Filter templated against C++ standard library
	 */
	template<class T>
	class SC_ABFilter
	{
	public:
		SC_ABFilter(units::time::second_t FilterTime, units::time::second_t ScanTime)
		{
			this->_tau = FilterTime.value();
			this->_scanT = ScanTime.value();
		}

		~SC_ABFilter()
		{
			;
		}

		T Filter(T PV)
		{
			T pvf = PV; // Filtered input value

			if(this->_tau != this->_scanT)
			{
				this->_beta = this->_scanT / (this->_tau + this->_scanT);
				this->_alpha = 1 - this->_beta;

				pvf = (this->_alpha * PV_out) + (this->_beta * PV);

				PV_out = std::abs(pvf) < 1E-9 ? 0.0 : pvf;

				return PV_out;

			}

			return pvf;
		}
	private:
		double _tau, _scanT;
		double _alpha, _beta;
		T PV_out;

	};

		/*
		Discrete Edge Detection Triggers
	*/

	/**
	 * @brief   Rising edge trigger. Takes boolean input signal, CLK,
	 *          in function call. The output, Q, will be TRUE for a single
	 *          evaluation when CLK goes from FALSE to TRUE. 
	 *          
	 *          Q will return to false on the next subsequent call.
	 */
	class R_TRIG
	{
	public:
		R_TRIG() { lastState = false; };

		void Check(bool CLK) 
		{ 
			this->Q = CLK && !this->lastState;
			this->lastState = CLK;
		}

		bool Q;

	private:
		bool state, lastState;

	};

	/**
	 * @brief   Falling edge trigger. Takes boolean input signal, CLK,
	 *          in function call. The output, Q, will be TRUE for a single
	 *          evaluation when CLK goes from TRUE to FALSE. 
	 * 
	 *          Q will return to false on the next subsequent call.
	 */
	class F_TRIG
	{
	public:
		F_TRIG() { lastState = false; };

		void Check(bool CLK) 
		{ 
			this->Q = !CLK && this->lastState;
			this->lastState = CLK;
		}

		bool Q;

	private:
		bool lastState;

	};
}

#endif //SC_DATATYPES_H
