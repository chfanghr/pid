//
// Created by 方泓睿 on 2019/11/18.
//

#include "pid.h"

#include <stdlib.h>
#include <math.h>

PID PIDNew() {
  PID result = malloc(sizeof(struct _PID));
  PIDInit(result);
  return result;
}

void PIDFree(PID pid) {
  if (pid != NULL)
	free(pid);
}

PIDError PIDGetError(PID pid) {
  if (pid)
	return pid->error;
  return kPIDFreed;
}

void PIDClearError(PID pid) {
  if (!pid)
	return;
  pid->error = kPIDOk;
}

void PIDInit(PID pid) {
  if (!pid)
	return;
  PIDSetGains(pid, 1, 0, 0);
  pid->integrator = pid->previous_error = 0;
  pid->frequency = 1;
  pid->integrator_limit = INFINITY;
  pid->error = kPIDOk;
}

void PIDSetGains(PID pid, double kp, double ki, double kd) {
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

void PIDGetGains(PID pid, double *kp, double *ki, double *kd) {
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

void PIDSetIntegralLimit(PID pid, double max) {
  if (!pid)
	return;
  if (isnan(max)) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  pid->integrator_limit = max;
}

double PIDGetIntegralLimit(PID pid) {
  if (!pid)
	return NAN;
  return pid->integrator_limit;
}

double PIDGetIntegral(PID pid) {
  if (!pid)
	return NAN;
  return pid->integrator;
}

void PIDResetIntegral(PID pid) {
  if (!pid)
	return;
  pid->integrator = 0;
}

void PIDSetFrequency(PID pid, double frequency) {
  if (!pid)
	return;
  if (isnan(frequency)) {
	pid->error = kPIDInvalidArgument;
	return;
  }
  pid->frequency = frequency;
}

double PIDGetFrequency(PID pid) {
  if (!pid)
	return NAN;
  return pid->frequency;
}

double PIDProcess(PID pid, double error) {
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