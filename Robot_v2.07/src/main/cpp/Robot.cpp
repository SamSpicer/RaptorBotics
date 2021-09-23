 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h" //includes the Robot header file for us to referance

#include <iostream> //standard cpp include to have normal cpp funtionalities

#include <frc/smartdashboard/SmartDashboard.h> //includes the header file for the smartdashboard that is used to potray data

#include <frc/VictorSP.h> //includes a header file that caould control the Victor speed controllers but not in use

#include <frc/Joystick.h> //includes a header file that allows us to make a joysstick object to use

#include <frc/DigitalInput.h> //includes a header file to take digital Inputs

#include <frc/PWMVictorSPX.h> //includes a header file for us to create objects to contorle the victor speed controllers (in use)

#include <frc/Preferences.h> //includes a header file to be able to create preferences

#include <cmath> //includes a standerd funtionality of cpp to do math 

#include <cameraserver/CameraServer.h> //includes a header file for camera server so that we are able to create a camera that we can see from

#include <frc/DigitalSource.h> // honestly do I don't remember what they are used for 

#include <frc/DriverStation.h> // honestly do I don't remember what they are used for

#include <frc/DigitalOutput.h> // honestly do I don't remember what they are used for

#include <frc/SPI.h>

#include <frc/ErrorBase.h> // honestly do I don't remember what they are used for

#include <ADIS16448_IMU.h> // I think this was the external source of code for the other gyroscope that we had

#include <frc/Timer.h> //includes a simple timer header file that can be used to create timers

#include <frc/WPIErrors.h> // honestly do I don't remember what they are used for

#include <hal/HAL.h> // honestly do I don't remember what they are used for

#include <frc/RobotDrive.h> 

#include <frc/drive/MecanumDrive.h>

#include <frc/AnalogGyro.h>

#include <chrono>

#include <time.h>

#include <frc/ADXRS450_Gyro.h>  

#include <networktables/NetworkTable.h>

#include <networktables/NetworkTableInstance.h> 

#include <frc/Filesystem.h>

#include <wpi/Path.h>

#include <wpi/SmallString.h>

#include <frc/trajectory/TrajectoryUtil.h>



// frc::ADXRS450_Gyro gyro(frc::SPI::Port::kMXP);

frc::ADIS16448_IMU imu{};


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  imu.Calibrate(); 
}

//declare all the veriables used by the function 

  //Creating drive motor objects start 

  frc::PWMVictorSPX m_rightFrontMotor{7};
  frc::PWMVictorSPX m_leftFrontMotor{6};
  frc::PWMVictorSPX m_rightBackMotor{8};
  frc::PWMVictorSPX m_leftBackMotor{9};

  //drive motor objects end 

  //Creating loader motor objects start 

  frc::PWMVictorSPX m_loder1{2};
  frc::PWMVictorSPX m_loder2{1};

  //Loader motor objects end

  frc::PWMVictorSPX m_elevator{0}; // Creating 
  frc::PWMVictorSPX m_intake{3};
  frc::PWMVictorSPX m_shooter1{4};
  frc::PWMVictorSPX m_shooter2{5};
  frc::Joystick m_mainJoystick{0};
  frc::Joystick m_Joystick_two{1};
  
  frc::DigitalInput m_limitSwitch{0}; //build limit switch object
  frc::DigitalInput m_limitSwitchLoader{1};
  frc::DigitalInput m_limitSwitchReset{9};
  frc::DigitalInput m_limitSwitchTopStop{2};


  frc::RobotDrive m_driver{m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor};


  //intake function 

  void Robot::intake(bool intakePressed){
    if(intakePressed){
      m_intake.Set(.75);
    }else{
      m_intake.Set(.000000000001);
    }
  }

  //loder (Loader) function

  void Robot::loder(double loderPressed ){
    if(loderPressed>0){
      m_loder1.Set(loderPressed);
      m_loder2.Set(loderPressed);
    }
  }
  void Robot::loader(bool trigger, double Speed){
  if(trigger){
    // timedLoader(3000);
    loder(Speed);
  }
  else {
    loder(0.00000000001);
  }
  }


  void Robot::loaderReverse(bool trigger , double speed){
    if(trigger){
      m_loder2.Set(speed);
    }else {
      m_loder2.Set(0.00000000001);
    }
  }

  void Robot::elevator(bool elevatorTriger, double speed){
    if(elevatorTriger){
      m_elevator.Set(speed);
    }else{
      m_elevator.Set(.0000000000001);
    }
  }

  void Robot::reverseelevator(bool reverseELevator){
    if(reverseELevator){
      m_elevator.Set(-.75);
    }else{
      m_elevator.Set(.0000000000001);
    }
  }

  void Robot::shooter(bool shooterTriger){
    if(shooterTriger){
      m_shooter1.Set(-1);
      m_shooter2.Set(-1);
    }else{
      m_shooter1.Set(.000000000001);
      m_shooter2.Set(.000000000001);
    }
  }

  void Robot::reverseShooter(bool shooterReverse){
    if(shooterReverse){
      m_shooter1.Set(-.75);
      m_shooter2.Set(-.75);
    }else{
      m_shooter1.Set(.000000000001);
      m_shooter2.Set(.000000000001);
    }
  }


// limelight 

double clamp(double in, double minval, double maxval){
  if(in>maxval) return maxval;
  if(in<minval) return minval;
  return in;
}

void Robot::Update_Limelight_Tracking(){

  const double STEER_K = 0.75;

  const double DRIVE_K = 0.75;

  
  const double DESIRED_TARGET_AREA = 10.0;
  const double MAX_DRIVE = 0.8;
  const double MAX_STEER = 1.0;


  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double tx = table ->GetNumber("tx", 0.0 );
  double ty = table ->GetNumber("ty", 0.0);
  double ta = table ->GetNumber("ta", 0.0);
  double tv = table ->GetNumber("tv", 0.0);

  if(tv < 1.0){
    m_LimelightHasTarget = false ;
    m_LimelightDriveCmd = 0.0;
    m_LimelightTurnCmd = 0.3;
  }
  else
  {
    m_LimelightHasTarget = true;

    //proportional steering
    m_LimelightTurnCmd = tx*STEER_K;
    if(m_LimelightTurnCmd < .25 ){
      
    }
    // m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER , MAX_STEER);

    //drive forward untill the target area reaches our desired area 

    m_LimelightDriveCmd = ty* DRIVE_K;
    // m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
  }
}

void Robot::LimeLightAutonomous(){
  
  
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double tx = table ->GetNumber("tx", 0.0 );
  double ty = table ->GetNumber("ty", 0.0);
  double ta = table ->GetNumber("ta", 0.0);
  double tv = table ->GetNumber("tv", 0.0);



}

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
  if(!frc::Preferences::GetInstance()->ContainsKey("Joy/loaderSpeed")){
    frc::Preferences::GetInstance()->PutDouble("Joy/loaderSpeed", .1);
  }
  if(!frc::Preferences::GetInstance()->ContainsKey("Joy/elevatorSpeed")){
    frc::Preferences::GetInstance()->PutDouble("Joy/elevatorSpeed", 1);
  }
}

void Robot::getPreferences(){
  minSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/Min Speed", .35);
  maxSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/Max Speed", .9);
  deadZone = frc::Preferences::GetInstance()->GetDouble("Joy/DeadZone", .05);
  elevatorSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/elevatorSpeed", .9);
  loaderSpeed = frc::Preferences::GetInstance()->GetDouble("Joy/loaderSpeed", .1);
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


//loader on a timer 


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
  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  //   // m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis);
  // } else {
  //   // Default Auto goes here
  //   // m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis);
  // }
wpi::SmallString<64> deployDirectory;
frc::filesystem::GetDeployDirectory(deployDirectory);
wpi::sys::path::append(deployDirectory, "paths");
wpi::sys::path::append(deployDirectory, "YourPath.wpilib.json");
frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);



}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic(){ 
  xAxis = m_mainJoystick.GetRawAxis(0);
  yAxis = m_mainJoystick.GetRawAxis(1);
  zAxis = m_mainJoystick.GetRawAxis(4);

  bool reverseElevator = m_mainJoystick.GetRawButton(3);
  // double buttonIntake = m_mainJoystick.GetRawAxis(2);
  bool buttonIntake = m_mainJoystick.GetRawButton(5);
  double buttonloader = m_mainJoystick.GetRawAxis(3);
  // bool elevatorTrigger = m_mainJoystick.GetRawButton(4);
  bool shooterButton = m_mainJoystick.GetRawButton(6);

  bool loaderRevolution = m_Joystick_two.GetRawButton(1);
  bool reverseloader = m_mainJoystick.GetRawButton(4);
  bool buttonIntake2 = m_Joystick_two.GetRawButton(3);

  bool do_limelight = m_Joystick_two.GetRawButtonPressed(2);

  bool gyroUse = m_mainJoystick.GetRawButtonPressed(1);

  checkKeys();
  getPreferences();
  if(fabs(xAxis)< deadZone ){
    xAxis = 0;
  }
  if(fabs(yAxis)< deadZone ){
    yAxis = 0;
  }
  xAxis = driveProfile(xAxis, minSpeed, maxSpeed);
  yAxis = driveProfile(yAxis, minSpeed, maxSpeed);
  // zAxis = driveProfile(zAxis,maxSpeed,minSpeed);
  double elevatorTurn = elevatorSpeed;
  double loaderspeed = loaderSpeed;

  if(do_limelight){
    Update_Limelight_Tracking();
    if(m_LimelightHasTarget){
      m_driver.MecanumDrive_Cartesian(m_LimelightDriveCmd,m_LimelightTurnCmd,0.0);
    }
    else
    {
      m_driver.MecanumDrive_Cartesian(0.0 , 0.0 , 0.0);
    }
  }
  else
  {    
    if(gyroUse){

      m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis,imu.GetGyroAngleY());
    }
    else
    {
      m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis);
    }
  }

 // m_driver.MecanumDrive_Cartesian(xAxis,yAxis,zAxis,heading);

  // if(loaderRevolution){
  //   // timedLoader(3000);
  //   loder(loaderspeed);
  // }
  // else {
  //   loder(0.00000000001);
  // }
  // if(reverseLoader){
  //   loder(-.5);
  // } else {
  //   loder(0.00000000001);
  // }
  // if(loaderRevolution){
  //   loader(loaderRevolution, loaderSpeed);
  // }else{
  //   if(reverseLoader){
  //     loader(reverseLoader, 1.0);
  //   }else{555555
  //     loder(0.00000000000001);
  //   }
  // }
  // loaderReverse(reverseLoader, loaderspeed );
  if(loaderRevolution){
    loader(loaderRevolution, loaderSpeed);
  }else {
    loader(true, .0000000000001);
  }


  // reverseelevator(elevatorTrigger);
  //limit switch start
  if(!m_limitSwitch.Get()){
    elevator(false,0.0);
    if(!m_limitSwitchReset.Get()){
      elevator(true, 1);
    }
  }else{
    elevator(reverseElevator, elevatorSpeed);
  }

  //limit switch end
  if(shooterButton){
    shooter(shooterButton);
    loader(true,.25);
  }else{
    shooter(false);
  }
  if(buttonIntake){
    intake(buttonIntake);
  }else if(buttonIntake2){
    intake(buttonIntake2);
  }else{
    intake(false);
  }
  if(m_limitSwitchLoader.Get() && !m_limitSwitchTopStop.Get()){
    loader(m_limitSwitchLoader.Get(), .5);
  }else{
    loder(buttonloader/2.0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
