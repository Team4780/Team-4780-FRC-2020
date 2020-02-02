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
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.vision.Cameras;
import frc.robot.vision.PixyCamera;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


// END IMPORTS

public class Robot extends TimedRobot {
// Port Instantiation
//  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final int leftVictorPort = 0;
  private static final int rightVictorPort = 1;
  private static final int elevatorSparkPort = 2;
  private static final int shooterSparkPort = 3;
  private static final int intakeSparkPort = 4;
  private static final int falconPort = 5;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;

// Button set-up
  private static final int bPowerCellIntake = 1;
  private static final int bShooterOuttake = 2;
  private static final int bHatchLevel2 = 7;
  private static final int bHatchLevel3 = 5;
  private static final int bDriveLevel = 2;
  private static final int bHomeLevel = 8;

// Drive VictorSP's
  VictorSP leftVictorSP = new VictorSP(leftVictorPort);
  VictorSP rightVictorSP = new VictorSP(rightVictorPort);
 
// Intake Spark
  Spark intakeSpark = new Spark(intakeSparkPort);  

// Elevator Spark
  Spark elevatorSpark = new Spark(elevatorSparkPort);

// Shooter Spark
  Spark shooterSpark = new Spark(shooterSparkPort);

// Falcon Shooter
 // TalonFX shooterFalcon = new TalonFX(falconPort);

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

// DriveTrain Creation
DifferentialDrive m_myRobot
= new DifferentialDrive(leftVictorSP, rightVictorSP);   

// Encoder Creation
  public static Encoder elevatorEncoder;

// Elevator Speed Setting
  double elevatorSpeedFast = -0.75;  //"fast" elevator speed (In the middle)
  double elevatorSpeedSlow = -0.4; //"slow" elevator speed (approaching hard stops)
  double elevatorSpeedStop = -0.25;
  double elevatorSpeedAct = 0; //speed elevator is actually set to (either the fast or slow) 

// Encoder Level Set up
  boolean elevatorButtonPressed = false;
  double targetDistance = 0;
  double homeLevel = 1.335;
  double hatchLevel2 = 12; 
  double hatchLevel3 = 25.5;
  double driveLevel = 3;

// Auto Choices in Shuffleboard
  private static final String kAutoLine = "Drive Straight - Auto Line";
  private static final String kAutoLineRight = "Drive Straight - Turn Right";
  private static final String kAutoLineLeft = "Drive Straight - Turn Left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Timer for autonomous
  public double timer = 0;

// Pixycam Creation
  private static Pixy2 pixy;
  
// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// Shooter Falcon Ramping Control
//  shooterFalcon.configOpenloopRamp(3.5);

// Encoder Instantiation
  elevatorEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
  elevatorEncoder.setDistancePerPulse((Math.PI * 1.804) / 192);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
    VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
      usbCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
  CameraServer camera2 = CameraServer.getInstance();
    VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
      usbCam2.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

// Gyro Cal
//  m_gyro.calibrate();

// PixyCam Initialization
  // Robot.pixy = Pixy2.createInstance(LinkType.SPI);
//  SmartDashboard.putBoolean("PixyCam Status:", pixy.getVersion() > 0);

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
// Post Encoder Distance to Shuffleboard
//  SmartDashboard.putNumber("Encoder Distance", elevatorEncoder.getDistance());
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
      // if (timer<timer+5){
      //   m_myRobot.tankDrive(0.25, -0.25);
      // }
      // else{
      //   m_myRobot.tankDrive(0, 0);
      // }
      //drivetrain run
      m_myRobot.tankDrive(1, 1);
      break;
// ---------------------
    case kAutoLineRight:
      if (Timer.getMatchTime()<timer+5){
        m_myRobot.tankDrive(0.25, -0.25);
      }
      else{
        m_myRobot.tankDrive(0, 0);
      }
      targetDistance = hatchLevel2;
      if (targetDistance == hatchLevel2) {
        intakeSpark.set(1);
      } 
      else {
        intakeSpark.set(0);
      }
      break;
// ---------------------
    case kAutoLineLeft:
      if (Timer.getMatchTime()<timer+5){
        m_myRobot.tankDrive(0.25, -0.25);
      }
      else{
        m_myRobot.tankDrive(0, 0);
      }
      targetDistance = hatchLevel3;
      if (targetDistance == hatchLevel3) {
        intakeSpark.set(1);
      } 
      else {
        intakeSpark.set(0);
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
//Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
  m_myRobot.arcadeDrive(m_joystick.getY()*0.8, m_joystick.getX()*0.8);
  // if(m_joystick.getRawButton(1))turned = true;
  // if(m_joystick.getPOV() != -1){
  // turned = false;
  // mustTurnDegree = m_joystick.getPOV();
  // }
  // if(!turned)turnDegrees(mustTurnDegree);

// Intake/Outtake Control Statements
  if (m_joystick2.getRawButton(bPowerCellIntake)) {
    intakeSpark.set(0.5);
  } 
  else {
    intakeSpark.set(0);
  }

// Shooter Spark Control Statement
  // if (m_joystick2.getRawButton(bShooterOuttake)) {
  //   shooterFalcon.set(ControlMode.MotionMagic, 1);
  // }
  // else {
  //   shooterFalcon.set(ControlMode.MotionMagic, 0);
  // }

// PixyCam Code
  // if(m_joystick2.getRawButton(1)){
  //   pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
  //   pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
  // }
  // else{
  //   pixy.setLamp((byte) 0, (byte) 0);
  //   pixy.setLED(0, 0, 0);  
  // }
//if(!isCamera)
//state = pixycam.init(1); // if no camera present, try to initialize
// isCamera = state>=0;

// SmartDashboard.putBoolean("Camera", isCamera);   //publish if we are connected
// pixycam.getCCC().getBlocks(false,255,255); //run getBlocks with arguments to have the camera
//                                            //acquire target data
// ArrayList<Block> blocks = pixycam.getCCC().getBlocks(); //assign the data to an ArrayList for convinience
// if(blocks.size() > 0) {
//   double xcoord = blocks.get(0).getX();       // x position of the largest target
//   double ycoord = blocks.get(0).getY();       // y position of the largest target
//   String data   = blocks.get(0).toString();   // string containing target info
//   SmartDashboard.putBoolean("present", true); // show there is a target present
//   SmartDashboard.putNumber("Xccord",xcoord);
//   SmartDashboard.putNumber("Ycoord", ycoord);
//   SmartDashboard.putString("Data", data );
// }
// else
//   SmartDashboard.putBoolean("present", false);
// SmartDashboard.putNumber("size", blocks.size()); //push to dashboard how many targets are detected
// if(RobotState.isEnabled() || RobotState.isDisabled()){
//   pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
//   pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
//   }
//   else{
  // pixy.setLamp((byte) 0, (byte) 0);
  // pixy.setLED(0, 0, 0);
  // } 
}
// END TELEOP PERIODIC


@Override
public void testPeriodic() {
}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT
