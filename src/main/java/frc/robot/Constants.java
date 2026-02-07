package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import swervelib.math.Matter;

public final class Constants {
    public static final Mass kRobotMass = Pounds.of(130.0);
    public static final Matter kRobotChassis =
            new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass.in(Kilograms));
    public static final Time kLoopTime = Seconds.of(0.13);
    public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(4.0);
    public static final Time kWheelLockTime = Seconds.of(10.0);

    public static final class ArmConstants {
        public static final int kLeftGearbox1 = 19;
        public static final int kLeftGearbox2 = 20;
        public static final int kRightGearbox1 = 8;
        public static final int kRightGearbox2 = 9;

        public static final int kTopLimitSwitchChannel = 4;
        public static final int kBottomLimitSwitchChannel = 1;

        public static final int kEncoderChannelA = 8;
        public static final int kEncoderChannelB = 9;

        public static final double kP = 0.04;
        public static final double kI = 0.04;
        public static final double kD = 0.0;
    }

    public static final class IntakeConstants {
        public static final int kTopShooterMotorID = 7;
        public static final int kBottomShooterMotorID = 3;

        public static final int kBumperIntakeMotorID = 4;
        public static final int kArmIntakeMotorID = 0;

        public static final int kIntakeLimitSwitchChannel = 0;
    }

    public static class OperatorConstants {
        public static final int kControllerPort = 0;
        public static final double kDeadband = 0.1;
        public static final double kTurnConstant = 3.0;
        public static final double kTranslationScale = 0.4593;
    }
}
