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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import edu.wpi.first.wpilibj.Encoder;


// END IMPORTS

public class Robot extends TimedRobot {

// Basic Function Instantiation

  // Drive VictorSP's
    VictorSP leftVictorSP = new VictorSP(leftVictorPort);
      public static final int leftVictorPort = 0;

    VictorSP rightVictorSP = new VictorSP(rightVictorPort);
      public static final int rightVictorPort = 1;

  // Intake Spark
    Spark intakeSpark = new Spark(intakeSparkPort);  
      public static final int intakeSparkPort = 2;

  // Uptake Sparks
    Spark uptakeSparkBottom = new Spark(uptakeSparkBottomPort);
      public static final int uptakeSparkBottomPort = 3;
    Spark uptakeSparkTop = new Spark(uptakeSparkTopPort);
      public static final int uptakeSparkTopPort = 4;

  // Falcon Shooter
    TalonFX shooterFalcon = new TalonFX(falconPort);
      public static final int falconPort = 5;
  
  // Spark Shooter (for testing purposes)
    Spark shooterSpark = new Spark(falconPort);

  // Winch Sparks
    Spark winchSparkLeft = new Spark(winchSparkLeftPort);
      public static final int winchSparkLeftPort = 6;
    Spark winchSparkRight = new Spark(winchSparkRightPort);
      public static final int winchSparkRightPort = 7;
  
  // Hook Elevator Sparks
    Spark hookElevatorLeftSpark = new Spark(hookElevatorSparkLeftPort);
      public static final int hookElevatorSparkLeftPort = 8;
    Spark hookElevatorRightSpark = new Spark(hookElevatorSparkRightPort);
      public static final int hookElevatorSparkRightPort = 9;

// Gyro
  //  public static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

// DriveTrain
  public static DriveTrain drivetrain;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Button set-up
  private static final int bPowerCellIntake = 1;
  private static final int bShooterOuttake = 3;

// Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;
//  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

//Joystick Instantiation
  public static Joystick m_joystick = new Joystick(kJoystickPort);
  public static Joystick m_joystick2 = new Joystick(kJoystick2Port);

// Auto Choices in Shuffleboard
  private static final String kInitLineShort = "Init Line, Short Run";
  private static final String kInitLineLong = "Init Line, Long Run";
  private static final String kInitLineShoot3 = "Init Line, Shoot 3";
  public String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// DriveTrain Creation
//  private DifferentialDrive m_myRobot = new DifferentialDrive(leftVictorSP, rightVictorSP); 

// Timer for autonomous
  public double timer = 0;
  Timer timer3;

// Encoder and Encoder Level Set up
  public static Encoder m_hookElevatorLeftEncoder;
  public static Encoder m_hookElevatorRightEncoder;
  boolean elevatorButtonPressed = false;
  double targetDistance = 0;
  double hookElevatorLeftLvl1 = 0;
  double hookElevatorLeftLvl2 = 0;
  double hookElevatorRightLvl1 = 0;
  double hookElevatorRightLvl2 = 0;

// Elevator Speeds
  double elevatorSpeedFast = -0.75;  // "Fast" elevator speed (In the middle)
  double elevatorSpeedSlow = -0.4; // "Slow" elevator speed (Approaching hard stops)
  double elevatorSpeedStop = -0.25;
  double elevatorSpeedAct = 0; // The speed the elevator is actually set to (either fast or slow) 


// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// DriveTrain
  drivetrain = new DriveTrain();

// Encoder Instantiation + Setup
  m_hookElevatorLeftEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
//  m_hookElevatorRightEncoder = new Encoder(5, 6, true, Encoder.EncodingType.k4X);
  m_hookElevatorLeftEncoder.setDistancePerPulse(Math.PI * 1.804 / 192);
//  m_hookElevatorRightEncoder.setDistancePerPulse(Math.PI * 1.804 / 192);

// Shooter Falcon Ramping Control
  shooterFalcon.configOpenloopRamp(3.5);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
    VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
      usbCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
  CameraServer camera2 = CameraServer.getInstance();
    VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
      usbCam2.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

// Gyro Calibration
  //  m_gyro.calibrate();

// Creating Dropdown Choices in Shuffleboard
  m_chooser.setDefaultOption("Init Line, Short Run", kInitLineShort);
  m_chooser.addOption("Init Line, Long Run", kInitLineLong);
  m_chooser.addOption("Init Line, Shoot 3", kInitLineShoot3);
  SmartDashboard.putData("Auto Chooser", m_chooser);

// Falcon Velocity Control
  shooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  shooterFalcon.setSensorPhase(true);
  /* Config the peak and nominal outputs */
  shooterFalcon.configNominalOutputForward(0, Constants.kTimeoutMs);
  shooterFalcon.configNominalOutputReverse(0, Constants.kTimeoutMs);
  shooterFalcon.configPeakOutputForward(1, Constants.kTimeoutMs);
  shooterFalcon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
   /* Config the Velocity closed loop gains in slot0 */
  shooterFalcon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
  shooterFalcon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
  shooterFalcon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
  shooterFalcon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
}

// END ROBOT INIT METHOD


@Override
public void robotPeriodic() {
//  SmartDashboard.putNumber("Encoder Hook-Left Distance", m_hookElevatorLeftEncoder.getDistance());
//  SmartDashboard.putNumber("Encoder Hook-Right Distance", m_hookElevatorRightEncoder.getDistance());
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
    case kInitLineShort:
    default:
    if (Timer.getMatchTime() < timer + 3) {
    //  m_myRobot.tankDrive(0.25, -0.25);
    }
    else {
    //  m_myRobot.tankDrive(0, 0);
    }
      break;
// ---------------------
    case kInitLineLong:
    if (Timer.getMatchTime() < timer + 5) {
    //  m_myRobot.tankDrive(0.25, -0.25);
    }
    else {
    //  m_myRobot.tankDrive(0, 0);
    }
      break;
// ---------------------
    case kInitLineShoot3:
    if (Timer.getMatchTime() < timer + 2) {
    //  m_myRobot.tankDrive(0.25, -0.25);
    }
    else {
    //  m_myRobot.tankDrive(0, 0);
    }
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
// DriveTrain
  drivetrain.drive(m_joystick);

// Intake/Outtake Control Statements
  if (m_joystick2.getRawButton(bPowerCellIntake)) {
   intakeSpark.set(0.5);
  } 
  else {
    intakeSpark.set(0);
 }

// Uptake Y-Axis
  if (m_joystick2.getY() < 0 || m_joystick2.getY() > 0) {

    if (m_joystick2.getY() < 0) {
      uptakeSparkBottom.set(0.5);
  }
    else {
    uptakeSparkBottom.set(0);
  }
    if (m_joystick2.getY() > 0) {
      uptakeSparkTop.setSpeed(0.5);
      }
    else {
    uptakeSparkTop.set(0);
  }
}

// Winch Control Statements
  if(m_joystick2.getRawAxis(6) == -1) {
    winchSparkLeft.setSpeed(0.5);
  }
  else {
    winchSparkLeft.set(0);
  }
  if(m_joystick2.getRawAxis(6) == 1) {
    winchSparkRight.setSpeed(0.5);
  }
  else {
    winchSparkRight.set(0);
  }

// Hook Elevator Control Statements
  boolean endgameActive = false;
  if(Timer.getMatchTime() == 120) {
  endgameActive = true;
  }
  else {
  endgameActive = false;
  }
 
 if(endgameActive = true) {

  // if(Math.abs(m_hookElevatorLeftEncoder.getDistance()-targetDistance) < 2){
  //   elevatorSpeedAct = elevatorSpeedSlow;
  // }
  // else{
  //   elevatorSpeedAct = elevatorSpeedFast;
  // }

  // if(Math.abs(m_hookElevatorRightEncoder.getDistance()-targetDistance) < 2){
  //   elevatorSpeedAct = elevatorSpeedSlow;
  // }
  // else{
  //   elevatorSpeedAct = elevatorSpeedFast;
  // }

  // boolean elevatorButtonPressed = (m_joystick2.getRawButton(5) || m_joystick2.getRawButton(6) || m_joystick2.getRawAxis(3) == 1 || m_joystick2.getRawAxis(3) == -1);

  // if(m_joystick2.getRawAxis(3) == 1) {
  //   targetDistance = hookElevatorLeftLvl1;
  // }
  // if(m_joystick2.getRawAxis(3) == -1) {
  //   targetDistance = hookElevatorRightLvl1;
  // }  
  // if(m_joystick2.getRawButton(5)) {
  //   targetDistance = hookElevatorLeftLvl2;
  // }
  // if(m_joystick2.getRawButton(6)) {
  //   targetDistance = hookElevatorRightLvl2;
  // }
  // if(elevatorButtonPressed) {

  // boolean TooLowLeft = (m_hookElevatorLeftEncoder.getDistance() - targetDistance) < -0.2;
  //   boolean TooLowRight = (m_hookElevatorRightEncoder.getDistance() - targetDistance) < -0.2;
  // boolean TooHighLeft = (m_hookElevatorLeftEncoder.getDistance()-targetDistance) > 0.2;
  //   boolean TooHighRight = (m_hookElevatorRightEncoder.getDistance() - targetDistance) > 0.2;
    
  // if (TooLowLeft) {
  //   hookElevatorLeftSpark.set(elevatorSpeedAct);
  // }
  // else if (TooHighLeft){
  //   hookElevatorLeftSpark.set(-elevatorSpeedAct*0.5);
  // }
  // else {
  //   hookElevatorLeftSpark.set(elevatorSpeedStop);
  // }

  // if (TooLowRight) {
  //   hookElevatorRightSpark.set(elevatorSpeedAct);
  // }
  // else if (TooHighRight){
  //   hookElevatorRightSpark.set(-elevatorSpeedAct*0.5);
  // }
  // else {
  //   hookElevatorRightSpark.set(elevatorSpeedStop);
  // }
  // }
  // }
  // else {
  // }

// Shooter Control Statements
  if(m_joystick2.getRawButton(bShooterOuttake)){
  shooterFalcon.set(ControlMode.Position, 1);
  }
  else {
  shooterFalcon.set(ControlMode.Position, 0);
  }

  if(m_joystick2.getRawButton(bShooterOuttake)){
    shooterSpark.set(1);
    }
    else {
    shooterSpark.set(0);
    }
  }
// Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
//   if(m_joystick.getRawButton(1))turned = true;
//   if(m_joystick.getPOV() != -1){
//   turned = false;
//   mustTurnDegree = m_joystick.getPOV();
//   }
//   if(!turned)turnDegrees(mustTurnDegree);
}
// END TELEOP PERIODIC


@Override
public void testPeriodic() {
  }
}

// END TEST PERIODIC METHOD & ROBOT PROJECT