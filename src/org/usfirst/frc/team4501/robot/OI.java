package org.usfirst.frc.team4501.robot;

import org.usfirst.frc.team4501.robot.commands.arm.ArmClose;
import org.usfirst.frc.team4501.robot.commands.arm.ArmOpen;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArmBottom;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArmTop;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsLow;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearHigh;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArm4Score;
import org.usfirst.frc.team4501.robot.commands.lift.LiftRobot;
import org.usfirst.frc.team4501.robot.commands.lift.StopLift;
import org.usfirst.frc.team4501.robot.commands.shooter.Intake;
import org.usfirst.frc.team4501.robot.commands.shooter.IntakeStop;
import org.usfirst.frc.team4501.robot.commands.shooter.Shoot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static final int TRIGGER = 1, BUTTON_2 = 2, BUTTON_3 = 3, BUTTON_4 = 4, BUTTON_5 = 5, BUTTON_6 = 6,
			BUTTON_7 = 7, BUTTON_8 = 8, BUTTON_9 = 9, BUTTON_10 = 10, BUTTON_11 = 11;

	XboxController controller = new XboxController(0);
	Joystick stick = new Joystick(1);

	Button shiftHigh = new JoystickButton(controller, controller.BUTTON_A);
	Button shiftLow = new JoystickButton(controller, controller.BUTTON_B);
	Button shoot = new JoystickButton(stick, BUTTON_2);
	Button liftBot = new JoystickButton(stick, BUTTON_3);
	Button armOpen = new JoystickButton(stick, BUTTON_4);
	Button armClose = new JoystickButton(stick, BUTTON_5);
	Button intake = new JoystickButton(stick, TRIGGER);
	Button armUp = new JoystickButton(stick, BUTTON_6);
	Button armSetup = new JoystickButton(stick, BUTTON_7);
	Button armDown = new JoystickButton(stick, BUTTON_8);

	public OI() {
		// Arm Commands
		armSetup.whenPressed(new PositionArm4Score());
		armUp.whenPressed(new PositionArmTop());
		armDown.whenPressed(new PositionArmBottom());
		armOpen.whenPressed(new ArmOpen());
		armClose.whenPressed(new ArmClose());

		// Drivetrain Commands
		shiftHigh.whenPressed(new ShiftGearsLow());
		shiftLow.whenPressed(new ShiftGearHigh());

		// Shooter Commands
		shoot.whileHeld(new Shoot());
		shoot.whenReleased(new IntakeStop());
		intake.whileHeld(new Intake());
		intake.whenReleased(new IntakeStop());

		// Lift Commands
		liftBot.whileHeld(new LiftRobot());
		liftBot.whenReleased(new StopLift());
	}

	// Returns the input from the left X axis on the Xbox controller.
	public double getLeftXboxX() {
		return controller.getRawAxis(0);
	}

	// Returns the input from the right Y axis on the Xbox controller.
	public double getLeftXboxY() {
		return controller.getRawAxis(1);
	}

	// Returns the input from the right X axis on the Xbox controller.
	public double getRightXboxX() {
		return controller.getRawAxis(4);
	}

	// Returns the input from the right Y axis on the Xbox controller.
	public double getRightXboxY() {
		return controller.getRawAxis(5);
	}

	// Returns the input from the X axis on the joystick.
	public double getShooterX() {
		return stick.getX();
	}

	// Returns the input from the Y axis on the joystick.
	public double getShooterY() {
		return stick.getY();
	}

	// Returns the input from the Z axis on the joystick.
	public double getShooterThrottle() {
		return stick.getZ();
	}

	// Returns the input from the triggers on the Xbox controller, by
	// subtracting the left and right values we can get the right trigger to act
	// as the forward input, in this case a throttle, and the right to be a
	// backward input, in this case a brake.
	public double getTriggers() {
		return controller.getRawAxis(XboxController.TRIGGER_R) - controller.getRawAxis(XboxController.TRIGGER_L);
	}

}
