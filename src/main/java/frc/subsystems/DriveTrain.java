  
package frc.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;


public class DriveTrain extends Subsystem {
	private VictorSP leftVictorSP;
	private VictorSP rightVictorSP;
	public double timer = 0;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public DriveTrain() {
		rightVictorSP = new VictorSP(Robot.rightVictorPort);
		leftVictorSP = new VictorSP(Robot.leftVictorPort);
	}
	
	public void drive(Joystick stick) {
		double x = stick.getX();
		double y = stick.getY();
		
		leftVictorSP.set(x-y);
		rightVictorSP.set(x+y);
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