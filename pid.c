//
// Created by 方泓睿 on 2019/11/18.
//

#include "pid.h"

#include <stdlib.h>
#include <math.h>

PIDController PIDNew() {
  PIDController result = malloc(sizeof(struct _PIDController));
  PIDInit(result);
  return result;
}

void PIDFree(PIDController pid) {
  if (pid != NULL)
	free(pid);
}

PIDError PIDGetError(PIDController pid) {
  if (pid)
	return pid->error;
  return kPIDFreed;
}

void PIDClearError(PIDController pid) {
  if (!pid)
	return;
  pid->error = kPIDOk;
}

void PIDInit(PIDController pid) {
  if (!pid)
	return;
  PIDSetGains(pid, 1, 0, 0);
  pid->integrator = pid->previous_error = 0;
  pid->frequency = 1;
  pid->integrator_limit = INFINITY;
  pid->error = kPIDOk;
}

void PIDSetGains(PIDController pid, double kp, double ki, double kd) {
  if (!pid)
	return;
  if (isnan(kp) || isnan(ki) || isnan(kd)) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PIDGetGains(PIDController pid, double *kp, double *ki, double *kd) {
  if (!pid)
	return;
  if (!kp || !ki || !kd) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  *kp = pid->kp;
  *ki = pid->ki;
  *kd = pid->kd;
}

void PIDSetIntegralLimit(PIDController pid, double max) {
  if (!pid)
	return;
  if (isnan(max)) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  pid->integrator_limit = max;
}

double PIDGetIntegralLimit(PIDController pid) {
  if (!pid)
	return NAN;
  return pid->integrator_limit;
}

double PIDGetIntegral(PIDController pid) {
  if (!pid)
	return NAN;
  return pid->integrator;
}

void PIDResetIntegral(PIDController pid) {
  if (!pid)
	return;
  pid->integrator = 0;
}

void PIDSetFrequency(PIDController pid, double frequency) {
  if (!pid)
	return;
  if (isnan(frequency)) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  pid->frequency = frequency;
}

double PIDGetFrequency(PIDController pid) {
  if (!pid)
	return NAN;
  return pid->frequency;
}

double PIDProcess(PIDController pid, double error) {
  if (!pid)
	return NAN;
  if (isnan(error)) {
	pid->error = kPIDInvalidArgument;
	return NAN;
  }

  double output;
  pid->integrator += error;

  if (pid->integrator > pid->integrator_limit) {
	pid->integrator = pid->integrator_limit;
  } else if (pid->integrator < -pid->integrator_limit) {
	pid->integrator = -pid->integrator_limit;
  }

  output = -pid->kp * error;
  output += -pid->ki * pid->integrator / pid->frequency;
  output += -pid->kd * (error - pid->previous_error) * pid->frequency;

  pid->previous_error = error;
  return output;
}