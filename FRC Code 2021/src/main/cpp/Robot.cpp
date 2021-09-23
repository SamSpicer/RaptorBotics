/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/VictorSP.h>

#include <frc/Joystick.h>

#include <frc/DigitalInput.h>

#include <frc/PWMVictorSPX.h>

#include <frc/Preferences.h>

#include <cmath>

#include <cameraserver/CameraServer.h>

#include <frc/DigitalInput.h>

#include <frc/DigitalSource.h>

#include <frc/DriverStation.h>

#include <frc/DigitalOutput.h>

#include <frc/DigitalSource.h>

#include <frc/DigitalInput.h>

#include <frc/SPI.h>

#include <frc/ErrorBase.h>

#include <ADIS16448_IMU.h>

#include <frc/Timer.h>

#include <frc/WPIErrors.h>

#include <hal/HAL.h>

#include <frc/RobotDrive.h>

#include <frc/drive/MecanumDrive.h>

#include <frc/AnalogGyro.h>



void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
}

//declare all the veriables used by the function 

  frc::PWMVictorSPX m_rightFrontMotor{7};
  frc::PWMVictorSPX m_leftFrontMotor{6};
  frc::PWMVictorSPX m_rightBackMotor{8};
  frc::PWMVictorSPX m_leftBackMotor{9};
  frc::PWMVictorSPX m_loder1{2};
  frc::PWMVictorSPX m_loder2{1};
  frc::PWMVictorSPX m_intake{3};
  frc::PWMVictorSPX m_shooter1{4};
  frc::PWMVictorSPX m_shooter2{5};
  frc::Joystick m_mainJoystick{0};

  frc::RobotDrive m_driver{m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor};

  frc::AnalogGyro m_gyro{0};

  // double xAxis = m_mainJoystick.GetRawAxis(0);
  // double yAxis = m_mainJoystick.GetRawAxis(1);
  // double zAxis = m_mainJoystick.GetRawAxis(2);
  // bool buttonIntake = m_mainJoystick.GetRawButton(1);
  // bool buttonShooter = m_mainJoystick.GetRawButton(2);

  void Robot::forward_backward(double yvalue){

      m_rightBackMotor.Set(-yvalue);
      m_rightFrontMotor.Set(yvalue);
      m_leftBackMotor.Set(yvalue);
      m_leftFrontMotor.Set(yvalue);
  }

  void Robot::left_right(double xvalue){
      m_leftFrontMotor.Set(xvalue);
      m_leftBackMotor.Set(-xvalue);
      m_rightFrontMotor.Set(-xvalue);
      m_rightBackMotor.Set(-xvalue);       
  }

  // void Robot::topRightCorner(double xvalue, double yvalue){
  //   m_leftFrontMotor.Set(xvalue);
  //   m_rightFrontMotor.Set(yvalue);
  //   m_leftBackMotor.Set(yvalue);
  //   m_rightBackMotor.Set(-xvalue);
  // }
  // void Robot::topLeftCorner(double xvalue, double yvalue){
  //   m_leftFrontMotor.Set(yvalue);
  //   m_rightFrontMotor.Set(xvalue);
  //   m_leftBackMotor.Set(yvalue);
  //   m_rightBackMotor.Set(-xvalue);
  // }
  // void Robot::backRightCorner(double xvalue, double yvalue){
  // m_leftFrontMotor.Set(xvalue);
  // m_rightFrontMotor.Set(yvalue);   
  // m_leftBackMotor.Set(yvalue);
  // m_rightBackMotor.Set(-xvalue);
  // }
  // void Robot::backLeftCorner(double xvalue, double yvalue){
  //   m_leftFrontMotor.Set(yvalue);
  //   m_rightFrontMotor.Set(xvalue);
  //   m_leftBackMotor.Set(yvalue);
  //   m_rightBackMotor.Set(-xvalue);
  // }
  void Robot::turn(double zvalue){
    // double inverseZ = -zvalue;
    if(zvalue >.1) {
      m_leftFrontMotor.Set(zvalue);
      m_rightFrontMotor.Set(-zvalue);
      m_leftBackMotor.Set(-zvalue);
      m_rightBackMotor.Set(-zvalue);
    }
    if (zvalue <.1){
      m_leftFrontMotor.Set(zvalue);
      m_rightFrontMotor.Set(-zvalue);
      m_leftBackMotor.Set(-zvalue);
      m_rightBackMotor.Set(-zvalue); 
    }
    
  }

  void Robot::driveSystem(double xValue , double yValue , double zValue){
    double deadZoneX = 0.1;
    double deadZoneY = 0.4;
    double deadZoneZ = 0.4;
    // if(zValue< deadZoneZ || zValue> deadZoneZ){
    //   turn(zValue);
    // }
    // if(xValue> deadZoneX && yValue> deadZoneY){
    //   topRightCorner(xValue,yValue);
    // }if(xValue> deadZoneX && yValue< deadZoneY){
    //   topLeftCorner(xValue,yValue);
    // }if(xValue< deadZoneX && yValue> deadZoneY){
    //   backRightCorner(xValue,yValue);
    // }if(xValue< deadZoneX && yValue< deadZoneY){
    //   backLeftCorner(xValue,yValue);
    // }
    // if(xValue> deadZoneX || xValue< deadZoneX ){
      left_right(xValue);
    // }
    //  else if(yValue> deadZoneY || yValue< deadZoneY){
      forward_backward(yValue);
    // }
  }


  //intake function 

  void Robot::intake(double intakePressed){
    if(intakePressed>0){
      m_intake.Set(intakePressed);
    }
    // while(intakePressed){
    //   m_intake.Set(.75);
    //   if(intakePressed==false){
    //     m_intake.Set(0);
    //     return;
    //   }
    // }
  }

  //loder the buttonsmight need to be changed

  void Robot::loder(double loderPressed ){
    if(loderPressed>0){
      m_loder1.Set(-loderPressed);
      m_loder2.Set(-loderPressed);
    }
    // while (loderPressed)
    // {
    //   m_loder1.Set(.75);
    //   m_loder2.Set(.75);
    //   if(loderPressed == false){
    //     m_loder1.Set(0);
    //     m_loder2.Set(0);
    //     return;
    //   }
    // }
    
  }

// gyro



//preferences and setting the speed 

void Robot::checkKeys(){
  if(!frc::Preferences::GetInstance()->ContainsKey("Joy/Min Speed")){
    frc::Preferences::GetInstance()->PutDouble("Joy/Min Speed", .35);
  }
  if(!frc::Preferences::GetInstance()->ContainsKey("Joy/Max Speed")){
    frc::Preferences::GetInstance()->PutDouble("Joy/Max Speed", 1);
  }
  if(!frc::Preferences::GetInstance()->ContainsKey("Joy/DeadZone")){
    frc::Preferences::GetInstance()->PutDouble("Joy/DeadZone", .05);
  }
}

void Robot::getPreferences(){
  minSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/Min Speed", .35);
  maxSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/Max Speed", 1);
  deadZone = frc::Preferences::GetInstance()->GetDouble("Joy/DeadZone", .05);
}

double Robot::driveProfile(double input, double min, double max){
  if(input == 0){
    return 0;
  }

  double absolute = fabs(input);
  double output = (max-min) * absolute + min;
  
  if(input < 0 ){
    return -output;
  }else{
    return output;
  }
}



/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
    m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis, m_gyro.GetAngle());
  }
}

void Robot::TeleopInit() {
  m_gyro.Calibrate();
}

void Robot::TeleopPeriodic(){ 
  xAxis = m_mainJoystick.GetRawAxis(0);
  yAxis = m_mainJoystick.GetRawAxis(1);
  zAxis = m_mainJoystick.GetRawAxis(4);
  double buttonIntake = m_mainJoystick.GetRawAxis(2);
  double buttonShooter = m_mainJoystick.GetRawAxis(3);
  double angle = m_gyro.GetAngle();
  // double error = m_gyro.GetError();
  // double buttonIntake = m_mainJoystick.GetTrigger(3);
  // double buttonShooter = m_mainJoystick.GetTrigger(4);
   checkKeys();
  if(fabs(xAxis)< deadZone ){
    xAxis = 0;
  }if(fabs(yAxis)< deadZone ){
    yAxis = 0;
  }
  xAxis = driveProfile(xAxis, minSpeed, maxSpeed);
  yAxis = driveProfile(yAxis, minSpeed, maxSpeed);
  // xAxis = -xAxis;
  // yAxis = -yAxis;
  // driveSystem(xAxis,yAxis,zAxis);

  m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis,angle);

  intake(buttonIntake);
  loder(buttonShooter);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
