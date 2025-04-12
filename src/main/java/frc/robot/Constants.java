// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = Units.lbsToKilograms(130);
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // Seconds, 20ms + 110ms Spark Max Velocity Lag.
    public static final double MAX_SPEED = Units.feetToMeters(10.0); // Maximum speed of robot in meters per second, used to limit acceleration.

    public static final class ArmConstants {
        public static final SparkMax leftGearbox1 = new SparkMax(19, MotorType.kBrushless);
        public static final SparkMax leftGearbox2 = new SparkMax(20, MotorType.kBrushless);
        public static final SparkMax rightGearbox1 = new SparkMax(8, MotorType.kBrushless);
        public static final SparkMax rightGearbox2 = new SparkMax(9, MotorType.kBrushless);

        public static final DigitalInput topLimitSwitch = new DigitalInput(4);
        public static final DigitalInput bottomLimitSwitch = new DigitalInput(1);

        public static final Encoder primaryNeckEncoder = new Encoder(8, 9);
        // public static final Encoder secondaryNeckEncoder = new Encoder(6, 7);
        
        public static final PIDConstants ARM_PID = new PIDConstants(0.004, 0, 0.0014);
        public static final double PID_THRESHOLD = 5;

        public enum ArmStates {
            INTAKE,
            SUBWOOFER,
            YEET,
            AMP
        }
    }

    public static final class IntakeConstants {
        public static final PWMSparkMax shooterTopMotor = new PWMSparkMax(1);
        public static final PWMSparkMax shooterBottomMotor = new PWMSparkMax(3);

        public static final PWMSparkMax bumperIntakeMotor = new PWMSparkMax(4);
        public static final PWMSparkMax beakIntakeMotor = new PWMSparkMax(0);
        
        public static final DigitalInput intakeLimitSwitch = new DigitalInput(0);
    }

    public static final class WristConstants {
        public static final SparkMax leftWristMotor = new SparkMax(0, MotorType.kBrushless);
        public static final SparkMax rightWristMotor = new SparkMax(0, MotorType.kBrushless);

        public static final Encoder wristEncoder = new Encoder(0, 0);
    
        public static final double swivelSpeed = 0.25;
        public static final double rotateSpeed = 0.3;
    }

    public static final class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0.0, 0.01);

        public static final boolean DRIVE_WITH_VISION = true;
    }

    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int AUXILIARY_CONTROLLER_PORT = 1;

        public static final double DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
        public static final double SCALE_TRANSLATION = 1.0;
    }
}