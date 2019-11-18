#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"
//
// Created by 方泓睿 on 2019/11/18.
//

#include "pid.h"
#include <gtest/gtest.h>

class PIDTestGroup : public testing::Test {
 public:
  PID::Controller controller;
};

TEST_F(PIDTestGroup, IntegralValueIsZeroAtInit) {
  auto res = controller.GetIntegral();
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 0.);
}

TEST_F(PIDTestGroup, ControllerGainsAtInit) {
  auto res = controller.GetGains();
  ASSERT_EQ(std::get<3>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 1.);
  ASSERT_DOUBLE_EQ(std::get<1>(res), 0.);
  ASSERT_DOUBLE_EQ(std::get<2>(res), 0.);
}

TEST_F(PIDTestGroup, ControllerFrequencyAtInit) {
  auto res = controller.GetFrequency();
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 1.);
}

TEST_F(PIDTestGroup, CanSetGains) {
  controller.SetGains(10., 20., 30.);
  auto res = controller.GetGains();
  ASSERT_EQ(std::get<3>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 10.);
  ASSERT_DOUBLE_EQ(std::get<1>(res), 20.);
  ASSERT_DOUBLE_EQ(std::get<2>(res), 30.);
}

TEST_F(PIDTestGroup, CanSetIntegraLimit) {
  controller.SetIntegralLimit(100.);
  auto res = controller.GetIntegralLimit();
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 100.);
}

TEST_F(PIDTestGroup, CanSetFrequency) {
  controller.SetFrequency(100.);
  auto res = controller.GetFrequency();
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 100.);
}

TEST_F(PIDTestGroup, ZeroErrorMakesForZeroOutput) {
  auto res = controller.Process(0.);
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), 0.);
}

TEST_F(PIDTestGroup, ByDefaultIsIdentity) {
  auto res = controller.Process(42);
  ASSERT_EQ(std::get<1>(res), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res), -42.);
}

TEST_F(PIDTestGroup, KpHasInfluenceOnOutput) {
  auto res_1 = controller.SetGains(2., 0., 0.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.Process(21.);
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), -42.0);
}

TEST_F(PIDTestGroup, IntegratorIsChangedByProcess) {
  auto res_1 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_1), PID::kPIDOk);
  auto res_2 = controller.GetIntegral();
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), 42.);
  auto res_3 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  auto res_4 = controller.GetIntegral();
  ASSERT_EQ(std::get<1>(res_4), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_4), 84.);
}

TEST_F(PIDTestGroup, AddingKiMakesOutputIncrease) {
  auto res_1 = controller.SetGains(0., 2., 0.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), -84);
  auto res_3 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), -168);
}

TEST_F(PIDTestGroup, IntegratorWorksInNegativeToo) {
  auto res_1 = controller.SetGains(0., 1., 0.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.Process(-42.);
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), 42);
  auto res_3 = controller.Process(-42.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), 84);
}

TEST_F(PIDTestGroup, AddingKdCreatesDerivativeAction) {
  auto res_1 = controller.SetGains(0., 0., 2.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), -84.0);
  auto res_3 = controller.Process(42.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), 0.0);
}

TEST_F(PIDTestGroup, IntegratorMaxValueIsRespected) {
  auto res_1 = controller.SetIntegralLimit(20.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.SetGains(0., 1., 0.);
  ASSERT_EQ(std::get<0>(res_2), PID::kPIDOk);
  auto res_3 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), -20.);
  auto res_4 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_4), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_4), -20.);
}

TEST_F(PIDTestGroup, IntegratorMaxValueWorksInNegativeToo) {
  auto res_1 = controller.SetIntegralLimit(20.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.SetGains(0., 1., 0.);
  ASSERT_EQ(std::get<0>(res_2), PID::kPIDOk);
  auto res_3 = controller.Process(-20.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), 20.);
  auto res_4 = controller.Process(-20.);
  ASSERT_EQ(std::get<1>(res_4), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_4), 20.);
}

TEST_F(PIDTestGroup, CanResetIntegrator) {
  auto res_1 = controller.SetGains(0., 1., 0.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_2), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_2), -20.);
  auto res_3 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), -40.);
  auto res_4 = controller.ResetIntegral();
  ASSERT_EQ(std::get<0>(res_4), PID::kPIDOk);
  auto res_5 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_5), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_5), -20.);
}

TEST_F(PIDTestGroup, FrequencyChangeIntegrator) {
  auto res_1 = controller.SetFrequency(10.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.SetGains(0., 1., 0.);
  ASSERT_EQ(std::get<0>(res_2), PID::kPIDOk);
  auto res_3 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), -2.);
  auto res_4 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_4), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_4), -4.);
}

TEST_F(PIDTestGroup, FrequencyChangeDerivative) {
  auto res_1 = controller.SetFrequency(10.);
  ASSERT_EQ(std::get<0>(res_1), PID::kPIDOk);
  auto res_2 = controller.SetGains(0., 0., 1.);
  ASSERT_EQ(std::get<0>(res_2), PID::kPIDOk);
  auto res_3 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_3), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_3), -200.);
  auto res_4 = controller.Process(20.);
  ASSERT_EQ(std::get<1>(res_4), PID::kPIDOk);
  ASSERT_DOUBLE_EQ(std::get<0>(res_4), 0.);
}

#pragma clang diagnostic pop