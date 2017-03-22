package org.usfirst.frc.team4501.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public static class Motors {
		
		public static final int LEFT_CAN_MOTOR_MASTER = 10;
		public static final int LEFT_CAN_MOTOR_SLAVE = 11;
		public static final int RIGHT_CAN_MOTOR_MASTER = 14;
		public static final int RIGHT_CAN_MOTOR_SLAVE = 15;
		
		public static final int LIFT = 5;

		public static final int ARM = 3;
		
		public static final int FLYWEEL = 3;
		public static final int INTAKE = 4;
	}

	public static class Solenoids {
		public static final int HIGHGEAR = 2;
		public static final int LOWGEAR = 3;
		public static final int ARMHIGH = 0;
		public static final int ARMLOW = 1;
	}
	
	public static class Sensors {
		public static final int ARM_HIGH = 0;
		public static final int ARM_LOW = 1;
		public static final int SHOOTER_ENCODER_A = 2, SHOOTER_ENCODER_B = 3;
	}

}
