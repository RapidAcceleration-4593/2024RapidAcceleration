// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (125) * 0.453592; // 10 lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class NeckConstants {
    public static final CANSparkMax leftGearbox1 = new CANSparkMax(19, MotorType.kBrushless);
    public static final CANSparkMax leftGearbox2 = new CANSparkMax(20, MotorType.kBrushless);
    public static final CANSparkMax rightGearbox1 = new CANSparkMax(8, MotorType.kBrushless);
    public static final CANSparkMax rightGearbox2 = new CANSparkMax(9, MotorType.kBrushless);
    public static final DigitalInput topLimitSwitch = new DigitalInput(4);
    public static final DigitalInput bottomLimitSwitch = new DigitalInput(1);
    public static final Encoder primaryNeckEncoder = new Encoder(2, 3);
    public static final Encoder secondaryNeckEncoder = new Encoder(6, 7);
  }

  public static final class BeakConstants {
    public static final PWMSparkMax bumperIntakeMotor = new PWMSparkMax(4);
    public static final PWMSparkMax beakIntakeMotor = new PWMSparkMax(0);
    public static final PWMSparkMax shooterTopMotor = new PWMSparkMax(1);
    public static final PWMSparkMax shooterBottomMotor = new PWMSparkMax(3);
    public static final DigitalInput intakeLimitSwitch = new DigitalInput(0);
  }

  public static final class ClimberConstants {
    public static final PWMSparkMax climberMotor = new PWMSparkMax(2);
  }

  public static final class LEDConstants {
    public static final Spark SiccLEDs = new Spark(5);
  }

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0.0, 0.01);
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.15;
    public static final double TURN_CONSTANT    = 6;
  }
}
