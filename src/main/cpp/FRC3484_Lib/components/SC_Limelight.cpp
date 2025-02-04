#include "FRC3484_Lib/components/SC_Limelight.h"

#include <cmath>
// #include "numbers"
#include <numbers>

using namespace SC;
using namespace nt;
using namespace std;

SC_Limelight::SC_Limelight(double Angle, double LensHeight)
{
	this->inst = NetworkTableInstance::GetDefault().GetTable("limelight");
	angle = Angle;
	lensHeight = LensHeight;
}

SC_Limelight::~SC_Limelight()
{

}

bool SC_Limelight::HasTarget()
{
	return ((bool)this->inst->GetNumber("tv", 0.0));
}

double SC_Limelight::GetOffsetX()
{
	return this->inst->GetNumber("tx", 0.0);
}

// Reports it degress (-29.8, 29.8)
double SC_Limelight::GetOffsetY()
{
	return this->inst->GetNumber("ty", 0.0);
}

double SC_Limelight::GetTargetArea()
{
	return this->inst->GetNumber("ta", 0.0);
}

double SC_Limelight::GetSkew()
{
	return this->inst->GetNumber("ts", 0.0);
}

double SC_Limelight::GetPipelineLatency()
{
	return this->inst->GetNumber("tl", 0.0);
}

double SC_Limelight::GetBBShort()
{
	return this->inst->GetNumber("tshort", 0.0);
}

double SC_Limelight::GetBBLong()
{
	return this->inst->GetNumber("tlong", 0.0);
}

double SC_Limelight::GetBBWidth()
{
	return this->inst->GetNumber("thor", 0.0);
}

double SC_Limelight::GetBBHeight()
{
	return this->inst->GetNumber("tvert", 0.0);
}

// double SC_Limelight::GetDistanceFromTarget()
// {
// 	return (this->targetHeight - this->lensHeight) / tan((this->angle + this->GetOffsetY()) * (std::numbers::pi_v<double> / 180.0));
// }

double SC_Limelight::GetDistanceFromTarget()
{
	return (this->targetHeight - this->lensHeight) / tan((this->angle + this->GetOffsetY()) * (std::numbers::pi_v<double> / 180.0));
}

int SC_Limelight::GetActivePipeline()
{
	return ((int)this->inst->GetNumber("getpipe", 0));
}

void SC_Limelight::SetLEDMode(SC_LEDMode Mode)
{
	this->inst->PutNumber("ledMode", Mode);
}

void SC_Limelight::SetDriverCam()
{
	this->inst->PutNumber("camMode", 1);
}

void SC_Limelight::SetVisionTracking()
{
	this->inst->PutNumber("camMode", 0);
}

void SC_Limelight::SetPipeline(int Pipeline)
{
	this->inst->PutNumber("pipeline", Pipeline);
}

void SC_Limelight::SetStreamMode(SC::SC_StreamMode Mode)
{
	this->inst->PutNumber("stream", Mode);
}

void SC_Limelight::SetCameraAngle(double Angle)
{
	this->angle = Angle;
}

void SC_Limelight::SetLensHeight(double Height)
{
	this->lensHeight = Height;
}

void SC_Limelight::SetTargetHeight(double Height)
{
	this->targetHeight = Height;
}