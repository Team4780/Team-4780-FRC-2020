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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


// END IMPORTS

public class Robot extends TimedRobot {
// Port Instantiation
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private static final int leftVictorPort = 0;
  private static final int rightVictorPort = 1;
  private static final int elevatorSparkPort = 2; // (to be changed to Falcon 500 on competition bot)
  private static final int shooterSparkPort = 3;
  private static final int intakeSparkPort = 4;
  private static final int indexingSparkPort = 5;

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

// Indexing Spark
  Spark indexingSpark = new Spark(indexingSparkPort);

// Elevator Spark
  Spark elevatorSpark = new Spark(elevatorSparkPort);

// Shooter Spark
  Spark shooterSpark = new Spark(shooterSparkPort);

// Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

//Joystick Instantiation
  private Joystick m_joystick = new Joystick(kJoystickPort);
  private Joystick m_joystick2 = new Joystick(kJoystick2Port);

// DriveTrain Creation
  private DifferentialDrive m_myRobot
    = new DifferentialDrive(leftVictorSP, rightVictorSP);    

// Encoder Creation
  public static Encoder elevatorEncoder;

// Pneumatic's Creation
//  public static Compressor compressor;
  public static DoubleSolenoid indexPiston;

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

// PixyCam Creation
  private Pixy2 pixycam;
  boolean isCamera = false;
  int state=-1;
  
// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);

// Encoder Instantiation
  elevatorEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
  elevatorEncoder.setDistancePerPulse((Math.PI * 1.804) / 192);

// Pneumatics Instantiation
//  compressor = new Compressor(1);
  indexPiston = new DoubleSolenoid(4, 5);

// Camera Instantiation (moved to Cameras.java)  
  Cameras.initCameras();

// Gyro
  m_gyro.calibrate();

// PixyCam Initialization
// (run pixycamera.java somehow)

//  pixycam.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
//  pixycam.setLED(255, 255, 255); // Sets the RGB LED to full white

// Creating Dropdown Choices in Shuffleboard
  m_chooser.setDefaultOption("Drive Straight - Auto Line", kAutoLine);
  m_chooser.addOption("Drive Straight - Turn Right", kAutoLineRight);
  m_chooser.addOption("Drive Straight - Turn Left", kAutoLineLeft);
  SmartDashboard.putData("Auto Chooser", m_chooser);
}

// END ROBOT INIT METHOD


@Override
public void robotPeriodic() {
// Post Encoder Distance to Shuffleboard
  SmartDashboard.putNumber("Encoder Distance", elevatorEncoder.getDistance());
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
      if (timer<timer+5){
        m_myRobot.tankDrive(0.25, -0.25);
      }
      else{
        m_myRobot.tankDrive(0, 0);
      }
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
//Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
  m_myRobot.arcadeDrive(m_joystick.getY()*0.8, m_joystick.getX()*0.8);

  if(m_joystick.getRawButton(1))turned = true;
  if(m_joystick.getPOV() != -1){
  turned = false;
  mustTurnDegree = m_joystick.getPOV();
  }
  if(!turned)turnDegrees(mustTurnDegree);

// Intake/Outtake Control Statements
  if (m_joystick2.getRawButton(bPowerCellIntake)) {
    intakeSpark.set(0.5);
  } 
  else {
    intakeSpark.set(0);
  }

// Shooter Spark Control Statement
  if (m_joystick2.getRawButton(bShooterOuttake)) {
    shooterSpark.set(1);
  }
  else {
    shooterSpark.set(0);
  }

boolean elevatorButtonPressed = (m_joystick2.getRawButton(bHomeLevel)) || (m_joystick2.getRawButton(bHatchLevel2)) || (m_joystick2.getRawButton(bHatchLevel3)) || (m_joystick.getRawButton(bDriveLevel));

//Encoded Elevator "Final" Code - Rishikesh & Kyle 3/21/19 (PA Day 1) - [WORKING 3/21/19]
if(Math.abs(elevatorEncoder.getDistance()-targetDistance) < 2){
  elevatorSpeedAct = elevatorSpeedSlow;
}
else{
  elevatorSpeedAct = elevatorSpeedFast;
}

double joystickYAxis = m_joystick2.getY();    
   
if (joystickYAxis>0 && elevatorEncoder.getDistance() < 1 || joystickYAxis<0 && elevatorEncoder.getDistance() > 26)
 {
   elevatorSpark.set(0); 
 }
else
 {
   if (m_joystick2.getRawButton(5) || m_joystick2.getRawButton(6) || m_joystick2.getRawButton(7) || m_joystick2.getRawButton(8)  || m_joystick.getRawButton(2)) 
   {
     
   }
   else{
     elevatorSpark.set(m_joystick2.getY()*0.5);
   }
  }

if(elevatorButtonPressed){

  boolean TooLow = (elevatorEncoder.getDistance()-targetDistance) < -0.2;
  boolean TooHigh = (elevatorEncoder.getDistance()-targetDistance) > 0.2;
  
    if (TooLow) {
      elevatorSpark.set(elevatorSpeedAct);
    }
    else if (TooHigh){
      elevatorSpark.set(-elevatorSpeedAct*0.5);
    }
    else {
      elevatorSpark.set(elevatorSpeedStop);
    }
  }
  else if (m_joystick2.getY() == 0){
    elevatorSpark.set(0);
  }

// PixyCam Code
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
}
// END TELEOP PERIODIC


@Override
public void testPeriodic() {
}
}

// END TEST PERIODIC METHOD & ROBOT PROJECT