//
// Created by 方泓睿 on 2019/11/18.
//

#ifndef PID__PID_H_
#define PID__PID_H_

#ifdef __cplusplus
#include <tuple>

#define PID_EXT extern "C"

namespace PID {
#else
#define PID_EXT
#endif

typedef enum { kPIDOk = 0, kPIDFreed, kPIDInvalidArgument } PIDError;

// Instance of a PID controller.
// Note that this structure is only public to be able to do static allocation of it.
// Do not access its fields directly.
struct _PID {
  double kp, ki, kd;
  double integrator;
  double previous_error;
  double integrator_limit;
  double frequency;
  PIDError error;
};

typedef struct _PID *PID;

// Allocates and initializes a new PID controller on the **heap**.
PID_EXT PID PIDNew();

// Frees the PID controller created by PIDNew.
PID_EXT void PIDFree(PID pid);

// Returns the last error occurred in operations.
PID_EXT PIDError PIDGetError(PID pid);

// Clear the error field.
PID_EXT void PIDClearError(PID pid);

// Initializes a PID controller.
PID_EXT void PIDInit(PID pid);

// Sets the gains of the given PID.
PID_EXT void PIDSetGains(PID pid, double kp, double ki, double kd);

// Returns the proportional gains of the controller.
PID_EXT void PIDGetGains(PID pid, double *kp, double *ki, double *kd);

// Sets a maximum value for the PID integrator
PID_EXT void PIDSetIntegralLimit(PID pid, double max);

// Returns the limit of the PID integrator.
PID_EXT double PIDGetIntegralLimit(PID pid);

// Returns the value of the PID integrator.
PID_EXT double PIDGetIntegral(PID pid);

// Resets the PID integrator to zero.
PID_EXT void PIDResetIntegral(PID pid);

// Process one step using the PID algorithm.
PID_EXT double PIDProcess(PID pid, double error);

// Sets the PID frequency for gain compensation.
PID_EXT void PIDSetFrequency(PID pid, double frequency);

// Returns the PID frequency for gain compensation.
PID_EXT double PIDGetFrequency(PID pid);

#ifdef __cplusplus
// Wrapper class
class Controller {
 private:
  _PID controller{};

  PIDError GetAndClearError();
 public:
  Controller();

  std::tuple<PIDError> Init();

  std::tuple<PIDError> SetGains(double kp, double ki, double kd);

  std::tuple<double, double, double, PIDError> GetGains();

  std::tuple<PIDError> SetIntegralLimit(double max);

  std::tuple<double, PIDError> GetIntegralLimit();

  std::tuple<double, PIDError> GetIntegral();

  std::tuple<PIDError> ResetIntegral();

  std::tuple<double, PIDError> Process(double error);

  double operator()(double error);

  std::tuple<PIDError> SetFrequency(double frequency);

  std::tuple<double, PIDError> GetFrequency();
};
}
#endif

#endif //PID__PID_H_
