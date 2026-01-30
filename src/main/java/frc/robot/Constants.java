// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
    public static final double kRobotMass = Units.lbsToKilograms(130);
    public static final Matter kRobotChassis = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass);
    public static final double kLoopTime = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag.
    public static final double kMaxVelocity = 4.0; // Maximum speed of robot in meters per second, used to limit acceleration.

    public static final class ArmConstants {
        public static final int kLeftGearbox1 = 19;
        public static final int kLeftGearbox2 = 20;
        public static final int kRightGearbox1 = 8;
        public static final int kRightGearbox2 = 9;

        public static final int kTopLimitSwitchChannel = 4;
        public static final int kBottomLimitSwitchChannel = 1;

        public static final int kEncoderChannelA = 8;
        public static final int kEncoderChannelB = 9;

        public static final double kP = 0.004;
        public static final double kI = 0.0;
        public static final double kD = 0.0; // 0.0014

        public enum ArmStates {
            INTAKE,
            SUBWOOFER,
            YEET,
            AMP
        }
    }

    public static final class IntakeConstants {
        public static final int kTopShooterMotorID = 1;
        public static final int kBottomShooterMotorID = 3;

        public static final int kBumperIntakeMotorID = 4;
        public static final int kArmIntakeMotorID = 0;

        public static final int kIntakeLimitSwitchChannel = 0;
    }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 1.0;
    }
}