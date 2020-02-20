package frc.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;


public class DriveTrain extends Subsystem {
	private VictorSP leftVictorSP;
	private VictorSP rightVictorSP;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public DriveTrain() {
		rightVictorSP = new VictorSP(Robot.rightVictorPort);
		leftVictorSP = new VictorSP(Robot.leftVictorPort);
	}
	
	public void drive(Joystick stick) {
		double x = stick.getX();
		double y = stick.getY();
		double xReduced = stick.getX()*0.3;
		double yReduced = stick.getY()*0.3;
		
		if(stick.getRawButton(2)){
		leftVictorSP.set(xReduced-yReduced);
		rightVictorSP.set(xReduced+yReduced);
		}
		else{
		leftVictorSP.set(x-y);
		rightVictorSP.set(x+y);
		}
	}
	public void auto() {
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	public static void drive(double y, double turningValue) {		
	}
}
