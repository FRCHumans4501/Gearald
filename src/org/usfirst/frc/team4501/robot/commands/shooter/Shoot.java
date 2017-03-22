package org.usfirst.frc.team4501.robot.commands.shooter;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Shoot extends Command {
	OI oi;
	Timer timer;

    public Shoot() {
    	requires(Robot.shooter);
    	
    	oi = Robot.oi;
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.shooter.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shooter.shoot();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooter.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
