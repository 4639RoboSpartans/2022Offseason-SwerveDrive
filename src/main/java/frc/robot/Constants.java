// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int DriverMotor1 = 1;
    public static final int RotaterMotor1 = 2;
    public static final int DriverMotor2 = 3;
    public static final int RotaterMotor2 = 4;
    public static final int DriverMotor3 = 5;
    public static final int RotaterMotor3 = 6;
    public static final int DriverMotor4 =7;
    public static final int RotaterMotor4 = 8;
	public static final int ModEncoder1 = 9;
	public static final int ModEncoder2 = 10;
	public static final int ModEncoder3 = 12;
	public static final int ModEncoder4 = 11;

    public static final int NUMBER_OF_CONTROLLERS = 2;
    public static final double DEADZONE_VALUE = 0.01;
	public static final double trackwidth = 0.44;
	public static final double wheelbase =0.44;

	
    public static final SwerveDriveKinematics kDriveKinematics;
	static{
		Translation2d translationFrontRight = new Translation2d(trackwidth / 2, wheelbase / 2);
		Translation2d translationFrontLeft = new Translation2d(-trackwidth / 2, wheelbase / 2);
		Translation2d translationBackRight = new Translation2d(trackwidth / 2, -wheelbase / 2);
		Translation2d translationBackLeft = new Translation2d(-trackwidth / 2, -wheelbase / 2);

		kDriveKinematics = new SwerveDriveKinematics(translationFrontRight, translationFrontLeft, translationBackLeft, translationBackRight);
	}

	public static final double ksVolts=0.65019;//0.69874;//0.66099;
	public static final double kvVoltSecondsPerMeter = 0.013915;//0.012955;//0.026728;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0033668;//0.0028624;//0.0028603;
	public static final double kPDriveVel = 0.12992;

	public static final double kRamseteB = 3.5;
    public static final double kRamseteZeta = 5;
	
	public static final double kMaxSpeedMetersPerSecond = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
	public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
	public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4 / 10;

    public enum Axes {
		LEFT_STICK_X(0), LEFT_STICK_Y(1), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_STICK_X(4), RIGHT_STICK_Y(5);

		private final int value;

		Axes(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum Buttons {
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
				7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

		private final int value;

		private Buttons(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

	}  
}
