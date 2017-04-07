package org.usfirst.frc.team4501.robot.subsystems;

import java.util.Arrays;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ArcadeDrive;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain.Kontours;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain.VisionMode;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain.VisionMove;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain.VisionRotate;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 */
public class DriveTrain extends Subsystem {
	// Rotation Constants.
	public static double rotateKp = 0.4;
	public static double rotateKi = .2;
	public static double rotateKd = 1.5;
	public static double maxRotateSpeed = .5;
	public static double maxRotateDuringMove = .45;
	public static double maxRotateError = 10;
	public static double maxRotateErrorDurringMove = 40;

	// Move Constants.
	public static double moveKp = 0.5;
	public static double moveKi = .02;
	public static double moveKd = 0.3;
	public static double maxMoveSpeed = .7;
	public static double visionMoveTargetWidth = 40;
	public static double visionMoveTargetWidthSlow = 25;

	// Vision Constants.
	public static double cameraWidth = 352;
	public static double cameraHeight = 240;
	public static double cameraCenterX = cameraWidth / 2.0;
	public static double cameraCenterY = cameraHeight / 2.0;
	private int targetOffsetX = 0;

	// Vision Modes
	// Rotate when the PID is rotating to center itself with the target.
	// Move when the PID is moving forward and getting closer to the target.
	// Done when the PID has detected that the it has met the target width.
	// Disabled when the Robot is disabled and nothing should happen.
	enum VisionMode {
		ROTATE, MOVE, DONE, DISABLED;
	}

	// Start the visionMode variable in Disabled mode.
	private VisionMode visionMode = VisionMode.DISABLED;

	// Combines the data inputs that we want together into one object.
	class Kontours implements Comparable<Kontours> {
		public double area;
		public double width;
		public double x;
		public double y;

		public Kontours(double area, double width, double x, double y) {
			this.area = area;
			this.width = width;
			this.x = x;
			this.y = y;
		}

		@Override
		public int compareTo(Kontours other) {
			// Descending Order
			return (int) Math.signum(other.area - area);
		}
	}

	// ---------------------------------------------------------------
	// R O T A T E
	// ---------------------------------------------------------------
	class VisionRotate extends PIDSubsystem {
		public VisionRotate(double p, double i, double d) {
			super("VisionRotate", p, i, d);
			getPIDController().setContinuous(false);
			getPIDController().setOutputRange(-maxRotateSpeed, maxRotateSpeed);
			setSetpoint(cameraCenterX);
			LiveWindow.addActuator("VisionRotate", "pid", getPIDController());
		}

		@Override
		public void enable() {
			super.enable();
			targetX = 0;
			targetOffsetX = 0;
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		protected double returnPIDInput() {
			// Calculate the distance between the center of the screen and the
			// center of the target
			if (Robot.instance.isAutonomous() || Robot.instance.isTest()) {
				targetX = centerX;

				// When close to the goal, change the target to be more to the
				// left
				// to help compensate for the difference in position between the
				// camera
				// and the peg reflector.
				if (centerWidth >= visionMoveTargetWidthSlow) {
					targetOffsetX = 60;
				}
			}
			return targetX;
		}

		@Override
		protected void usePIDOutput(double output) {
			switch (visionMode) {
			case ROTATE:
				if (period > 10 && Math.abs(rotateAvgError) < maxRotateError) {
					pidRotateOutput = 0;
					if (centerWidth >= visionMoveTargetWidthSlow) {
						visionMode = VisionMode.DONE;
					} else {
						visionMode = VisionMode.MOVE;
					}
				} else {
					pidRotateOutput = output;
				}
				break;

			default:
				pidRotateOutput = 0;
				break;
			}
		}
	}

	// ---------------------------------------------------------------
	// M O V E
	// ---------------------------------------------------------------
	class VisionMove extends PIDSubsystem {
		public VisionMove(double p, double i, double d) {
			super("VisionMove", p, i, d);
			this.setSetpoint(visionMoveTargetWidth);
			getPIDController().setContinuous(false);
			getPIDController().setOutputRange(0, maxMoveSpeed);
			LiveWindow.addActuator("VisionMove", "pid", getPIDController());
		}

		@Override
		protected double returnPIDInput() {
			switch (visionMode) {
			case MOVE:
				// if (centerWidth > visionMoveTargetWidth) {
				// visionMode = VisionMode.DONE;
				// }
				if (centerWidth >= visionMoveTargetWidthSlow) {
					visionMode = VisionMode.ROTATE;
				} else if ((Math.abs(targetX - cameraCenterX) > maxRotateErrorDurringMove)) {
					visionRotate.getPIDController().setOutputRange(-maxRotateDuringMove, maxRotateDuringMove);
					visionMode = VisionMode.ROTATE;
				}
				break;

			default:
				break;
			}
			return centerWidth;
		}

		@Override
		protected void usePIDOutput(double output) {
			switch (visionMode) {
			case MOVE:
				pidMoveOutput = output;
				break;

			default:
				pidMoveOutput = 0;
				break;
			}
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		public void disable() {
			super.disable();
			pidMoveOutput = 0;
		}

		@Override
		public void enable() {
			super.enable();
			visionMode = VisionMode.ROTATE;
		}
	}

	// ---------------------------------------------------------------
	// D R I V E T R A I N
	// ---------------------------------------------------------------

	RobotDrive driveTrain;
	CANTalon leftMasterTalon, leftSlaveTalon, rightMasterTalon, rightSlaveTalon;

	DoubleSolenoid shifterSolenoid;

	OI oi;

	VisionRotate visionRotate;
	VisionMove visionMove;

	public double centerY;
	public double centerX;
	public double centerWidth;

	private double targetX;
	private double rotateAvgError;
	public double pidMoveOutput;
	public double pidRotateOutput;

	public int period;

	public NetworkTable netTable;

	public double[] defaultValues = new double[4];

	public DriveTrain() {

		// Initializing the 4 motors that are on the drive system
		// Left Motors
		leftMasterTalon = new CANTalon(RobotMap.Motors.LEFT_CAN_MOTOR_MASTER);
		leftSlaveTalon = new CANTalon(RobotMap.Motors.LEFT_CAN_MOTOR_SLAVE);
		// Right Motors
		rightMasterTalon = new CANTalon(RobotMap.Motors.RIGHT_CAN_MOTOR_MASTER);
		rightSlaveTalon = new CANTalon(RobotMap.Motors.RIGHT_CAN_MOTOR_SLAVE);

		// Creating a master talon and a slave talon for each side so the motors
		// spin the correct way and don't break our gearboxes.
		// Left Motors
		leftSlaveTalon.changeControlMode(TalonControlMode.Follower);
		leftSlaveTalon.set(leftMasterTalon.getDeviceID());
		// Right Motors
		rightSlaveTalon.changeControlMode(TalonControlMode.Follower);
		rightSlaveTalon.set(rightMasterTalon.getDeviceID());

		// Initializing the RobotDrive controller by giving it the two master
		// talons for each side
		// the slaves will follow whatever the master talons are doing.
		driveTrain = new RobotDrive(leftMasterTalon, rightMasterTalon);

		// Initializing the solenoid that controls the shifters on the gearbox.
		shifterSolenoid = new DoubleSolenoid(RobotMap.Solenoids.HIGHGEAR, RobotMap.Solenoids.LOWGEAR);

		// Initializing new PID controller objects that the class was made for
		// up above with the constants that were also written above.
		visionRotate = new VisionRotate(rotateKp, rotateKi, rotateKd);
		visionMove = new VisionMove(moveKp, moveKi, moveKd);
	}

	// A method that a command can call on to give motors its values to move
	// with.
	public void arcadeDrive(double forward, double rotate) {
		driveTrain.arcadeDrive(-forward, rotate);
	}

	// Shifts the gears into Torque Mode
	public void shiftGearsHigh() {
		shifterSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	// Shifts the gears into Speed Mode
	public void shiftGearsLow() {
		shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	// Set a default command that always is running unless something else
	// requires the subsystem.
	public void initDefaultCommand() {
		setDefaultCommand(new ArcadeDrive());
	}

	// A method that initializes the camera, PID controllers, and other
	// variables.
	public void initPIDs() {
		netTable = NetworkTable.getTable("GRIP/myContoursReport");
		visionMode = VisionMode.ROTATE;
		period = 0;
		visionRotate.enable();
		visionMove.enable();
		centerWidth = Double.MIN_VALUE;
		getCenters();
	}

	// A method that is constantly called on, used for updating values and the mode that the PID is in.
	public void updatePIDPeriodic() {
		getCenters();
		++period;

		// Use exponential averaging to calc the target error.
		rotateAvgError = rotateAvgError * .80 + (centerX - cameraCenterX) * .20;

		switch (visionMode) {
		case ROTATE:
		case MOVE:
			if (centerWidth >= visionMoveTargetWidthSlow) {
				arcadeDrive((pidMoveOutput * .75), -pidRotateOutput);
			} else {
				arcadeDrive(pidMoveOutput, -pidRotateOutput);
			}
			break;

		default:
			arcadeDrive(0, 0);
			break;
		}

		System.out.printf(
				"%d Mode=%s Rotate=%.1f Move=%.1f targetX=%.1f Width=%.1f rotateAvgErr=%.1f targetOffSetX = %d\n",
				System.currentTimeMillis(), visionMode, pidRotateOutput, pidMoveOutput, targetX, centerWidth,
				rotateAvgError, targetOffsetX);
	}
	
	// A method that returns true when the PID is in Done mode, used to stop the PID from running more than it should.
	public boolean isDone() {
		if (visionMode == VisionMode.DONE) {
			System.out.println("----------------The Pid Is Done -------------------");
		}
		return (visionMode == VisionMode.DONE);
	}
	
	// A method that resets the target offset back to its initial state, used after the PID is done.
	public void onEnd() {
		targetOffsetX = 0;
		System.out.println("targetOffSetx == 0");
	}
	
	// A method that for safety makes sure that the PID doesn't run when it's not supposed to.
	public void disablePIDPeriodic() {
		visionMode = VisionMode.DISABLED;
	}
	
	// A method that reads from the data tables provided by GRIP and filters/organizes them and also updates said values.
	public boolean getCenters() {
		double[] tableX = netTable.getNumberArray("centerX", defaultValues);
		double[] tableY = netTable.getNumberArray("centerY", defaultValues);
		double[] tableWidth = netTable.getNumberArray("width", defaultValues);
		double[] tableArea = netTable.getNumberArray("area", defaultValues);

		// NetworkTables aren't updated atomically and therefore the lengths can
		// differ.
		int count = Math.min(tableX.length, tableY.length);
		count = Math.min(count, tableWidth.length);
		count = Math.min(count, tableArea.length);

		if (count == 0) {
			return false;
		}

		Kontours[] kontours = new Kontours[count];
		for (int i = 0; i < count; i++) {
			double area = tableArea[i];
			kontours[i] = new Kontours(area, tableWidth[i], tableX[i], tableY[i]);
		}

		// Between the two with the largest areas, use the rightmost contour.
		Arrays.sort(kontours);
		int targetIndex = 0;
		if (kontours.length > 1) {
			if (kontours[0].x < kontours[1].x) {
				targetIndex = 1;
			}
		}
		double newWidth = kontours[targetIndex].width;
		if (centerWidth == Double.MIN_VALUE) {
			centerWidth = newWidth;
		}

		centerX = kontours[targetIndex].x;

		centerX += targetOffsetX;
		centerWidth = newWidth;

		return true;
	}
}
