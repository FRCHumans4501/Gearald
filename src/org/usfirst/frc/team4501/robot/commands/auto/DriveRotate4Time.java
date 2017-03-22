package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveRotate4Time extends Command {
	double runTime;
	double rotateSpeed;

    public DriveRotate4Time(double runTime, double rotateSpeed) {
        requires(Robot.driveTrain);
        this.runTime = runTime;
        this.rotateSpeed = rotateSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.arcadeDrive(0, rotateSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (this.timeSinceInitialized() > runTime);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
