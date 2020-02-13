/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.vision.PixyCam;
import frc.subsystems.DriveTrain;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


// END IMPORTS

public class Robot extends TimedRobot {

// Basic Function Instantiation
  // Drive VictorSP's
    VictorSP leftVictorSP = new VictorSP(leftVictorPort);
      public static final int leftVictorPort = 0;

    VictorSP rightVictorSP = new VictorSP(rightVictorPort);
      public static final int rightVictorPort = 1;

  // Shooter Spark
    Spark shooterSpark = new Spark(shooterSparkPort); 
      public static final int shooterSparkPort = 3;

  // Intake Spark
    Spark intakeSpark = new Spark(intakeSparkPort);  
      public static final int intakeSparkPort = 4;

  // Falcon Shooter
  // TalonFX shooterFalcon = new TalonFX(falconPort);
  //  public static final int falconPort = 5;

// Gyro
  //  public static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

// DriveTrain
  public static DriveTrain drivetrain;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Button set-up
  private static final int bPowerCellIntake = 1;
  private static final int bShooterOuttake = 2;

// Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;
//  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

//Joystick Instantiation
  private Joystick m_joystick = new Joystick(kJoystickPort);
  private Joystick m_joystick2 = new Joystick(kJoystick2Port);

// Auto Choices in Shuffleboard
  private static final String kAutoLine = "Drive Straight - Auto Line";
  private static final String kAutoLineRight = "Drive Straight - Turn Right";
  private static final String kAutoLineLeft = "Drive Straight - Turn Left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Timer for autonomous
  public double timer = 0;

// Pixycam Creation
  boolean isCamera = false;
  int state=- 1;

  
// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// DriveTrain
  drivetrain = new DriveTrain();

// Shooter Falcon Ramping Control
  // RobotMap.shooterFalcon.configOpenloopRamp(3.5);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
    VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
      usbCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
  CameraServer camera2 = CameraServer.getInstance();
    VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
      usbCam2.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

// Gyro Cal
  //  m_gyro.calibrate();

// Creating Dropdown Choices in Shuffleboard
  m_chooser.setDefaultOption("Drive Straight - Auto Line", kAutoLine);
  m_chooser.addOption("Drive Straight - Turn Right", kAutoLineRight);
  m_chooser.addOption("Drive Straight - Turn Left", kAutoLineLeft);
  SmartDashboard.putData("Auto Chooser", m_chooser);

// Falcon Velocity Control
  // shooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  // shooterFalcon.setSensorPhase(true);
  //   /* Config the peak and nominal outputs */
  // shooterFalcon.configNominalOutputForward(0, Constants.kTimeoutMs);
  // shooterFalcon.configNominalOutputReverse(0, Constants.kTimeoutMs);
  // shooterFalcon.configPeakOutputForward(1, Constants.kTimeoutMs);
  // shooterFalcon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
  //   /* Config the Velocity closed loop gains in slot0 */
  // shooterFalcon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
  // shooterFalcon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
  // shooterFalcon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
  // shooterFalcon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
}

// END ROBOT INIT METHOD


@Override
public void robotPeriodic() {
}

// END ROBOT PERIODIC METHOD6


@Override
public void autonomousInit() {
  m_autoSelected = m_chooser.getSelected();
}

// END AUTONOMOUS INIT METHOD


@Override
public void autonomousPeriodic() {

// Code for auto choices
  switch (m_autoSelected) {
    case kAutoLine:
      default:
      break;
// ---------------------
    case kAutoLineRight:
      break;
// ---------------------
    case kAutoLineLeft:
      break;
// ---------------------
  }
}

// END AUTONOMOUS PERIODIC METHOD


//Gyro Math Method
// public void turnDegrees(int degree) {
//   if(turned)return;
//   angle = m_gyro.getAngle() % 360;
//   if(angle-10 > degree)m_myRobot.arcadeDrive(0.8, (angle - degree)*kP);
//   else if(angle+10 < degree)m_myRobot.arcadeDrive(0.8, (angle + degree)*kP);
//   else turned = true;
// }

// END GYRO MATH METHOD


@Override
public void teleopPeriodic() {
//Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
  // if(m_joystick.getRawButton(1))turned = true;
  // if(m_joystick.getPOV() != -1){
  // turned = false;
  // mustTurnDegree = m_joystick.getPOV();
  // }
  // if(!turned)turnDegrees(mustTurnDegree);

// Intake/Outtake Control Statements
 // if (m_joystick2.getRawButton(bPowerCellIntake)) {
 //   intakeSpark.set(0.5);
  //} 
  //else {
   // intakeSpark.set(0);
  // }

// Shooter Spark Control Statement
  // if (m_joystick2.getRawButton(bShooterOuttake)) {
  //   shooterFalcon.set(ControlMode.MotionMagic, 1);
  // }
  // else {
  //   shooterFalcon.set(ControlMode.MotionMagic, 0);
  // }
// Drivetrain
  drivetrain.drive(m_joystick);
 }
// END TELEOP PERIODIC


@Override
public void testPeriodic() {
}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT