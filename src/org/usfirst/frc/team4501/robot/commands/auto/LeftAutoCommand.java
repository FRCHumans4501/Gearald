package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.commands.arm.ArmClose;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArm4Score;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsHigh;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsLow;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LeftAutoCommand extends CommandGroup {

    public LeftAutoCommand() {
    	requires(Robot.driveTrain);
    	
    	addSequential(new PositionArm4Score());
    	addSequential(new ShiftGearsHigh());
    	addSequential(new DriveStraight4Time(1.25, .8));
    	addSequential(new ShiftGearsLow());
    	addSequential(new DriveRotate4Time(.35, .8));
    	addSequential(new AutoPIDEnable());
    	addSequential(new DriveStraight4Time(1, 0.40));
    	addSequential(new ArmClose());
    	addSequential(new DriveStraight4Time(.2, 0));
    	addSequential(new DriveStraight4Time(1, -0.5));
    }
}
