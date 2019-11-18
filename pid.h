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
struct _PIDController {
  double kp, ki, kd;
  double integrator;
  double previous_error;
  double integrator_limit;
  double frequency;
  PIDError error;
};

typedef struct _PIDController *PIDController;

// Allocates and initializes a new PID controller on the **heap**.
PID_EXT PIDController PIDNew();

// Frees the PID controller created by PIDNew.
PID_EXT void PIDFree(PIDController pid);

// Returns the last error occurred in operations.
PID_EXT PIDError PIDGetError(PIDController pid);

// Clear the error field.
PID_EXT void PIDClearError(PIDController pid);

// Initializes a PID controller.
PID_EXT void PIDInit(PIDController pid);

// Sets the gains of the given PID.
PID_EXT void PIDSetGains(PIDController pid, double kp, double ki, double kd);

// Returns the proportional gains of the controller.
PID_EXT void PIDGetGains(PIDController pid, double *kp, double *ki, double *kd);

// Sets a maximum value for the PID integrator
PID_EXT void PIDSetIntegralLimit(PIDController pid, double max);

// Returns the limit of the PID integrator.
PID_EXT double PIDGetIntegralLimit(PIDController pid);

// Returns the value of the PID integrator.
PID_EXT double PIDGetIntegral(PIDController pid);

// Resets the PID integrator to zero.
PID_EXT void PIDResetIntegral(PIDController pid);

// Process one step using the PID algorithm.
PID_EXT double PIDProcess(PIDController pid, double error);

// Sets the PID frequency for gain compensation.
PID_EXT void PIDSetFrequency(PIDController pid, double frequency);

// Returns the PID frequency for gain compensation.
PID_EXT double PIDGetFrequency(PIDController pid);

#ifdef __cplusplus
// Wrapper class
class Controller {
 private:
  _PIDController controller{};

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
