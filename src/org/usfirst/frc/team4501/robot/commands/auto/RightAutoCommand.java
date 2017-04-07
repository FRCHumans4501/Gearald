package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.commands.arm.ArmClose;
import org.usfirst.frc.team4501.robot.commands.arm.ArmOpen;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArm4Score;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsLow;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearHigh;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RightAutoCommand extends CommandGroup {

    public RightAutoCommand() {
    	requires(Robot.driveTrain);

    	addSequential(new PositionArm4Score());
    	addSequential(new ShiftGearsLow());
    	addSequential(new DriveStraight4Time(1.25, 1));
    	addSequential(new ShiftGearHigh());
    	addSequential(new DriveRotate4Time(.4, .8));
    	addSequential(new AutoPIDEnable());
    	addSequential(new DriveStraight4Time(1, 0.45));
    	addSequential(new ArmOpen());
    	addSequential(new DriveStraight4Time(.2, 0));
    	addSequential(new DriveStraight4Time(1, -0.5));
    }
}
