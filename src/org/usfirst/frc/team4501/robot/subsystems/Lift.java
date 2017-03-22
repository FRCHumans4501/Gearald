package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.lift.StopLift;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem {

	Talon liftTalon;

	public Lift() {
		// Initializes lift talon.
		liftTalon = new Talon(RobotMap.Motors.LIFT);
	}
	
	// A method that makes the lift motor spin so it can lift the robot up.
	public void liftRobot() {
		liftTalon.set(-.75);
	}
	
	// A method that stops the Lift from moving when it doesn't need to.
	public void stopLift() {
		liftTalon.set(0);
	}
	
	// 
    public void initDefaultCommand() {
    	setDefaultCommand(new StopLift());
    }
}

