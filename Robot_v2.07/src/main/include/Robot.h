/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void driveSystem(double, double,double);
  void topRightCorner(double, double);
  void topLeftCorner(double, double);
  void backRightCorner(double, double);
  void backLeftCorner(double, double);
  void turn(double);
  void left_right(double);
  void forward_backward(double);
  void intake(bool);
  void loder(double);
  void timedLoader(double);
  void elevator(bool, double);
  void loader(bool, double);
  void reverseelevator(bool);
  void shooter(bool);
  void reverseShooter(bool);
  void Update_Limelight_Tracking();
  void loaderReverse(bool, double);
  void LimeLightAutonomous();


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  double minSpeed;
  double maxSpeed;
  double deadZone;
  double loaderSpeed;
  double zAxis;
  double yAxis;
  double xAxis;
  double elevatorSpeed;
  double driveProfile(double, double, double);
  void checkKeys();
  void getPreferences();

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;
  
};
