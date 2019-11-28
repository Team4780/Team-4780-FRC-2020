/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// END IMPORTS

public class Robot extends TimedRobot {
// Port Instantiation
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final int leftVictorPort = 0;
  private static final int rightVictorPort = 1;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Drive VictorSP's
  VictorSP leftVictorSP = new VictorSP(leftVictorPort);
  VictorSP rightVictorSP = new VictorSP(rightVictorPort);

// New Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;

// Drivetrain (with VictorSP's)
  DifferentialDrive m_myRobot = new DifferentialDrive(leftVictorSP, rightVictorSP);
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);
  private Joystick m_joystick = new Joystick(kJoystickPort);
  private Joystick m_joystick2 = new Joystick(kJoystick2Port);

// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
  VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
  usbCam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
  CameraServer camera2 = CameraServer.getInstance();
  VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
  usbCam2.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);  

// Gyro Calibration 
  m_gyro.calibrate();
}

// END ROBOT INIT METHOD

@Override
public void robotPeriodic() {
}

// END ROBOT PERIODIC METHOD

@Override
public void autonomousInit() {
}

// END AUTONOMOUS INIT METHOD

@Override
public void autonomousPeriodic() {
}

// END AUTONOMOUS PERIODIC METHOD

//Gyro Math Method
public void turnDegrees(int degree) {
  if(turned)return;
  angle = m_gyro.getAngle() % 360;
  if(angle-10 > degree)m_myRobot.arcadeDrive(0.8, (angle - degree)*kP);
  else if(angle+10 < degree)m_myRobot.arcadeDrive(0.8, (angle + degree)*kP);
  else turned = true;
}

// END GYRO MATH METHOD

@Override
public void teleopPeriodic() {
//Gyro Math (tested & working as of 2/9/19)
  m_myRobot.arcadeDrive(m_joystick.getY()*0.8, m_joystick.getX()*0.8);

  if(m_joystick.getRawButton(1))turned = true;
  if(m_joystick.getPOV() != -1){
  turned = false;
  mustTurnDegree = m_joystick.getPOV();
  }
  if(!turned)turnDegrees(mustTurnDegree);
}

// END TELEOP PERIODIC

@Override
public void testPeriodic() {
}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT
