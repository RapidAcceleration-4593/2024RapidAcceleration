// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.*;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.BeakCommands.Intake.*;
import frc.robot.commands.BeakCommands.Shooter.*;
import frc.robot.commands.ElevatorCommands.MaintainElevatorLevel;
import frc.robot.commands.ElevatorCommands.RaiseElevatorToLevel;
import frc.robot.commands.PrimaryCommands.ManualControl.*;
import frc.robot.commands.PrimaryCommands.PresetPositions.*;
import frc.robot.commands.PrimaryCommands.Vision.*;

import com.pathplanner.lib.auto.NamedCommands;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final PrimarySubsystem primarySubsystem = new PrimarySubsystem();

  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController auxXbox = new CommandXboxController(1);

  public RobotContainer()
  {
    NamedCommands.registerCommand("IntakePosition", new IntakePosition(primarySubsystem));
    NamedCommands.registerCommand("SubwooferPosition", new SubwooferPosition(primarySubsystem));
    NamedCommands.registerCommand("VisionAngle", new VisionNeckAngleAuto(primarySubsystem));

    NamedCommands.registerCommand("Shooter", new ShooterAuto(primarySubsystem));
    NamedCommands.registerCommand("Intake", new IntakeAuto(primarySubsystem));
    NamedCommands.registerCommand("StopAll", new StopAllAuto(primarySubsystem));

    NamedCommands.registerCommand("ResetGyro", Commands.runOnce(drivebase::zeroGyro));

    configureBindings();

    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRightX() * 0.95);

    Command driveFieldOrientedAnglularVelocityAprilTagAlignment = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.95,
        () -> driverXbox.start().getAsBoolean(),
        primarySubsystem.returnCamera()
    );

    elevatorSubsystem.setDefaultCommand(new MaintainElevatorLevel(elevatorSubsystem));
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityAprilTagAlignment);
  }

  private void configureBindings()
  {
    /*BooleanSupplier elevatorBottomLSPressedSupplier = elevatorSubsystem::isBottomLimitSwitchPressed;
    Trigger elevatorBottomLSPressedTrigger = new Trigger(elevatorBottomLSPressedSupplier);
    elevatorBottomLSPressedTrigger.onTrue(elevatorSubsystem.resetEncoderCommand());

    BooleanSupplier elevatorTopLSPressedSupplier = elevatorSubsystem::isTopLimitSwitchPressed;
    Trigger elevatorTopLSPressedTrigger = new Trigger(elevatorTopLSPressedSupplier);
    elevatorTopLSPressedTrigger.onTrue(elevatorSubsystem.configureSetpointForTopOfElevatorCommand());
*/
    // Driver Controller
    driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

    //driverXbox.b().onTrue(elevatorSubsystem.togglePIDEnabledCommand());   
    //driverXbox.x().onTrue(elevatorSubsystem.resetEncoderCommand());

    driverXbox.a().whileTrue(elevatorSubsystem.moveElevatorDownCommand());        
    driverXbox.y().whileTrue(elevatorSubsystem.moveElevatorUpCommand());
    
    driverXbox.povUp().onTrue(new RaiseElevatorToLevel(elevatorSubsystem, ElevatorLevel.L1));
    driverXbox.povRight().onTrue(new RaiseElevatorToLevel(elevatorSubsystem, ElevatorLevel.L2));
    driverXbox.povDown().onTrue(new RaiseElevatorToLevel(elevatorSubsystem, ElevatorLevel.L3));
    driverXbox.povLeft().onTrue(new RaiseElevatorToLevel(elevatorSubsystem, ElevatorLevel.L4));
    
    // Auxilary Controller
    auxXbox.back().onTrue(new ManualControlEnabled(primarySubsystem));
    auxXbox.start().onTrue(new ManualControlDisabled(primarySubsystem));

    auxXbox.povDown().onTrue(new IntakePosition(primarySubsystem));
    auxXbox.povLeft().onTrue(new SubwooferPosition(primarySubsystem));
    auxXbox.povUp().onTrue(new AmpPosition(primarySubsystem));
    auxXbox.povRight().onTrue(new YeetPosition(primarySubsystem));
    auxXbox.b().whileTrue(new VisionNeckAnglePressed(primarySubsystem));
    auxXbox.b().whileFalse(new VisionNeckAngleNotPressed(primarySubsystem));

    auxXbox.rightBumper().whileTrue(new BeakShooter(primarySubsystem));
    auxXbox.rightBumper().whileFalse(new BeakShooterStop(primarySubsystem));

    auxXbox.leftBumper().whileTrue(new BeakIntake(primarySubsystem));
    auxXbox.leftBumper().whileFalse(new BeakIntakeStop(primarySubsystem));

    auxXbox.x().whileTrue(new BeakOuttake(primarySubsystem));
    auxXbox.x().whileFalse(new BeakIntakeStop(primarySubsystem));
  }

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand(PrimarySubsystem.autoName);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}