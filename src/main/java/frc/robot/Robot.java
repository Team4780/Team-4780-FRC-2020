/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Encoder;


// END IMPORTS

public class Robot extends TimedRobot {

// Basic Function Instantiation

  // Drive VictorSP's
  public static final int leftVictorPort = 0;
    public static VictorSP leftVictorSP = new VictorSP(leftVictorPort);

  public static final int rightVictorPort = 1;
    public static VictorSP rightVictorSP = new VictorSP(rightVictorPort);

  // Intake Spark
  VictorSP intakeSP = new VictorSP(intakeSPPort);  
    public static final int intakeSPPort = 2;

  // Uptake Sparks
  Spark uptakeSpark = new Spark(uptakeSparkPort);
    public static final int uptakeSparkPort = 3;

  // Falcon Shooter
  // TalonFX shooterFalcon = new TalonFX(falconPort);
  //   public static final int falconPort = 4;

  // Shooter Spark
  Spark shooterSpark = new Spark(shooterSparkPort);
    public static final int shooterSparkPort = 5;

  // Winch Sparks
  PWMVictorSPX winchSPXLeft = new PWMVictorSPX(winchSPXLeftPort);
    public static final int winchSPXLeftPort = 5;
  PWMVictorSPX winchSPXRight = new PWMVictorSPX(winchSPXRightPort);
    public static final int winchSPXRightPort = 6;

  // Hook Elevator Sparks
  PWMVictorSPX hookElevatorLeftSPX = new PWMVictorSPX(hookElevatorSPXLeftPort);
    public static final int hookElevatorSPXLeftPort = 7;
  PWMVictorSPX hookElevatorRightSPX = new PWMVictorSPX(hookElevatorSPXRightPort);
    public static final int hookElevatorSPXRightPort = 8;

  // Intake Drop Spark
  Spark intakeDropSpark = new Spark(intakeDropSparkPort);
    public static final int intakeDropSparkPort = 9;

  // LED Rings
   AddressableLED LED_Ring = new AddressableLED(6);
    AddressableLEDBuffer LED_RingBuffer;

// Gyro
  public static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

// DriveTrain
  public static DriveTrain drivetrain;

// Joystick Ports
  private static final int kJoystickPort = 0;
  private static final int kJoystick2Port = 1;
  private static final int kEndGameJoystickPort = 2;

// Button set-up
  private static final int bPowerCellIntake = 1;
  private static final int bShooterOuttake = 3;
  private static final int bHookElevatorLeftLvl0 = 0;
  private static final int bHookElevatorLeftLevel1 = 1;
  private static final int bHookElevatorLeftLevel2 = 4;
  private static final int bHookElevatorRightLvl0 = 0;
  private static final int bHookElevatorRightLevel1 = 3;
  private static final int bHookElevatorRightLevel2 = 2;
  private static final int bIntakeOut = 9;
  private static final int bIntakeIn = 10;

// Gyro Instantiation 
  int P, I, D = 1;
  private static final double kP = 0.005; // propotional turning constant
  double gyro_angle;
  boolean turned = true;
  int mustTurnDegree = 0;
  private static final double kAngleSetpoint = 0.0;
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

//Joystick Instantiation
  public static Joystick m_joystick = new Joystick(kJoystickPort);
  public static Joystick m_joystick2 = new Joystick(kJoystick2Port);
  public static Joystick m_endgameJoystick = new Joystick(kEndGameJoystickPort);

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
  double hookElevatorLeftLvl0 = 0;
  double hookElevatorLeftLvl1 = 2;
  double hookElevatorLeftLvl2 = 3;
  double hookElevatorRightLvl0 = 0;
  double hookElevatorRightLvl1 = 2;
  double hookElevatorRightLvl2 = 3;

// Elevator Speeds
  double elevatorSpeedFast = -0.75;  // "Fast" elevator speed (In the middle)
  double elevatorSpeedSlow = -0.4; // "Slow" elevator speed (Approaching hard stops)
  double elevatorSpeedStop = -0.25;
  double elevatorSpeedAct = 0; // The speed the elevator is actually set to (either fast or slow) 

// Chameleon Vision Network Table Creation
  NetworkTableInstance chameleon = NetworkTableInstance.create();
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = chameleon.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  public static NetworkTableEntry angle;
  public static NetworkTableEntry validAngle;


// END TIMED ROBOT METHOD

@Override
public void robotInit() {
// Joystick Creation
  m_joystick = new Joystick(0);
  m_joystick2 = new Joystick(1);
  m_endgameJoystick = new Joystick(2);

// DriveTrain
  drivetrain = new DriveTrain();

  // LED Ring
  LED_RingBuffer = new AddressableLEDBuffer(25);
  LED_Ring.setLength(LED_RingBuffer.getLength());
  for (var i = 0; i < LED_RingBuffer.getLength(); i++) {
    LED_RingBuffer.setRGB(i, 0, 0, 255);
 } 
 LED_Ring.start();
 LED_Ring.setData(LED_RingBuffer);

// Encoder Instantiation + Setup
 m_hookElevatorLeftEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
 m_hookElevatorRightEncoder = new Encoder(5, 6, true, Encoder.EncodingType.k4X);
 m_hookElevatorLeftEncoder.setDistancePerPulse(Math.PI * 1.804 / 192);
 m_hookElevatorRightEncoder.setDistancePerPulse(Math.PI * 1.804 / 192);

// Shooter Falcon Ramping Control
//  shooterFalcon.configOpenloopRamp(1.5);

// Camera Instantiation
  CameraServer camera = CameraServer.getInstance();
    VideoSource usbCam = camera.startAutomaticCapture("cam0", 0);
      usbCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
  CameraServer camera2 = CameraServer.getInstance();
    VideoSource usbCam2 = camera2.startAutomaticCapture("cam1", 1);
      usbCam2.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

// Gyro Calibration
  m_gyro.calibrate();

// Creating Dropdown Choices in Shuffleboard
  m_chooser.setDefaultOption("Init Line, Short Run", kInitLineShort);
  m_chooser.addOption("Init Line, Long Run", kInitLineLong);
  m_chooser.addOption("Init Line, Shoot 3", kInitLineShoot3);
  SmartDashboard.putData("Auto Chooser", m_chooser);
}

// END ROBOT INIT METHOD


@Override
public void robotPeriodic() {
 SmartDashboard.putNumber("Encoder Hook-Left Distance", m_hookElevatorLeftEncoder.getDistance());
 SmartDashboard.putNumber("Encoder Hook-Right Distance", m_hookElevatorRightEncoder.getDistance());
}

// END ROBOT PERIODIC METHOD


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
//   gyro_angle = m_gyro.getAngle() % 360;
//   if(gyro_angle-10 > degree) m_myRobot.arcadeDrive(0.8, (gyro_angle - degree)*kP);
//   else if(gyro_angle+10 < degree) m_myRobot.arcadeDrive(0.8, (gyro_angle + degree)*kP);
//   else turned = true;
// }

// END GYRO MATH METHOD


@Override
public void teleopPeriodic() {
// DriveTrain
  drivetrain.drive(m_joystick);

// Intake Control Statements
  if (m_joystick2.getRawButton(bPowerCellIntake)) {
   intakeSP.set(0.5);
  } 
  else {
   intakeSP.set(0);
 }

// Uptake Control Statements
  if (m_joystick2.getY() > 0) {
    if (m_joystick2.getY() < 0) {
      uptakeSpark.set(0.5);
    }
    else {
    uptakeSpark.set(0);
    }
}

// Intake In & Out Of Frame Perimeter
  if(m_joystick2.getRawButton(bIntakeOut) || m_joystick2.getRawButton(bIntakeIn)) {
    if(m_joystick2.getRawButton(bIntakeOut)) {
      intakeDropSpark.set(0.5);
     }
     if(m_joystick2.getRawButton(bIntakeIn)) {
       intakeDropSpark.set(-0.5);
     }
  }
  else {
    intakeDropSpark.set(0);
  }

// Winch Control Statements
  if(m_endgameJoystick.getRawButton(7)) {
    winchSPXLeft.set(0.5);
  }
  else {
    winchSPXLeft.set(0);
  }
  if(m_endgameJoystick.getRawButton(8)) {
    winchSPXRight.set(0.5);
  }
  else {
    winchSPXRight.set(0);
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

  if(Math.abs(m_hookElevatorLeftEncoder.getDistance()-targetDistance) < 2) {
    elevatorSpeedAct = elevatorSpeedSlow;
  }
  else {
    elevatorSpeedAct = elevatorSpeedFast;
  }

  if(Math.abs(m_hookElevatorRightEncoder.getDistance()-targetDistance) < 2) {
    elevatorSpeedAct = elevatorSpeedSlow;
  }
  else {
    elevatorSpeedAct = elevatorSpeedFast;
  }

  boolean elevatorButtonPressed = (m_endgameJoystick.getRawButton(1) || m_endgameJoystick.getRawButton(4) || m_endgameJoystick.getRawButton(3) || m_endgameJoystick.getRawButton(2));

  if(m_endgameJoystick.getRawButton(bHookElevatorLeftLvl0)) {
    targetDistance = hookElevatorLeftLvl0;
  }
  if(m_endgameJoystick.getRawButton(bHookElevatorLeftLevel1)) {
    targetDistance = hookElevatorLeftLvl1;
  }
  if(m_endgameJoystick.getRawButton(bHookElevatorLeftLevel2)) {
    targetDistance = hookElevatorLeftLvl2;
  }
  if(m_endgameJoystick.getRawButton(bHookElevatorRightLvl0)) {
    targetDistance = hookElevatorRightLvl0;
  } 
  if(m_endgameJoystick.getRawButton(bHookElevatorRightLevel1)) {
    targetDistance = hookElevatorRightLvl1;
  }
  if(m_endgameJoystick.getRawButton(bHookElevatorRightLevel2)) {
    targetDistance = hookElevatorRightLvl2;
  }

  if(elevatorButtonPressed) {
  boolean TooLowLeft = (m_hookElevatorLeftEncoder.getDistance() - targetDistance) < -0.2;
    boolean TooLowRight = (m_hookElevatorRightEncoder.getDistance() - targetDistance) < -0.2;
  boolean TooHighLeft = (m_hookElevatorLeftEncoder.getDistance()-targetDistance) > 0.2;
    boolean TooHighRight = (m_hookElevatorRightEncoder.getDistance() - targetDistance) > 0.2;
    
  if (TooLowLeft) {
    hookElevatorLeftSPX.set(elevatorSpeedAct);
  }
  else if (TooHighLeft) {
    hookElevatorLeftSPX.set(-elevatorSpeedAct*0.5);
  }
  else {
    hookElevatorLeftSPX.set(elevatorSpeedStop);
  }

  if (TooLowRight) {
    hookElevatorRightSPX.set(elevatorSpeedAct);
  }
  else if (TooHighRight) {
    hookElevatorRightSPX.set(-elevatorSpeedAct*0.5);
  }
  else {
    hookElevatorRightSPX.set(elevatorSpeedStop);
      }
    }
  }
  else {
  }

// Gyro Math (tested & working as of 2/9/19) (old math is commented out as of 1/6/20)
//   if(m_joystick.getRawButton(1))turned = true;
//   if(m_joystick.getPOV() != -1) {
//   turned = false;
//   mustTurnDegree = m_joystick.getPOV();
//   }
//   if(!turned)turnDegrees(mustTurnDegree);
// }
}
// END TELEOP PERIODIC


@Override
public void testPeriodic() {
  }
}

// END TEST PERIODIC METHOD & ROBOT PROJECT
