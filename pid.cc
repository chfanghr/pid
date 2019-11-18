//
// Created by 方泓睿 on 2019/11/18.
//

#include "pid.h"

#include <stdexcept>
#include <string>

namespace PID {
PIDError Controller::GetAndClearError() {
  auto err = PIDGetError((PID) &controller);
  PIDClearError((PID) &controller);
  return err;
}

Controller::Controller() { Init(); }

std::tuple<PIDError> Controller::Init() {
  PIDInit(&controller);
  return std::make_tuple(GetAndClearError());
}

std::tuple<PIDError> Controller::SetGains(double kp, double ki, double kd) {
  PIDSetGains((PID) &controller, kp, ki, kd);
  return std::make_tuple(GetAndClearError());
}

std::tuple<double, double, double, PIDError> Controller::GetGains() {
  double kp, ki, kd;
  PIDGetGains((PID) &controller, &kp, &ki, &kd);
  return std::make_tuple(kp, ki, kd, GetAndClearError());
}

std::tuple<PIDError> Controller::SetIntegralLimit(double max) {
  PIDSetIntegralLimit((PID) &controller, max);
  return std::make_tuple(GetAndClearError());
}

std::tuple<double, PIDError> Controller::GetIntegralLimit() {
  auto res = PIDGetIntegralLimit((PID) &controller);
  return std::make_tuple(res, GetAndClearError());
}

std::tuple<double, PIDError> Controller::GetIntegral() {
  auto res = PIDGetIntegral((PID) &controller);
  return std::make_tuple(res, GetAndClearError());
}

std::tuple<PIDError> Controller::ResetIntegral() {
  PIDResetIntegral((PID) &controller);
  return std::make_tuple(GetAndClearError());
}

std::tuple<double, PIDError> Controller::Process(double error) {
  auto res = PIDProcess((PID) &controller, error);
  return std::make_tuple(res, GetAndClearError());
}

double Controller::operator()(double error) {
  auto res = Process(error);
  if (std::get<1>(res))
	throw std::runtime_error("PID error occurred: " + std::to_string((int) std::get<1>(res)));
  return std::get<0>(res);
}

std::tuple<PIDError> Controller::SetFrequency(double frequency) {
  PIDSetFrequency((PID) &controller, frequency);
  return std::make_tuple(GetAndClearError());
}

std::tuple<double, PIDError> Controller::GetFrequency() {
  auto res = PIDGetFrequency((PID) &controller);
  return std::make_tuple(res, GetAndClearError());
}
}