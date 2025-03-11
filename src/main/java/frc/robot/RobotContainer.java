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
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.*;
import frc.robot.commands.arm.MaintainArmState;
import frc.robot.commands.arm.SetArmSetpoint;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ShootCommand;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem(s)
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Controller(s)
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController auxController = new CommandXboxController(1);

  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverController.getRightX() * 0.95);

  public RobotContainer() {
    configureBindings();

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    armSubsystem.setDefaultCommand(new MaintainArmState(armSubsystem));
  }

  private void configureBindings() {
    // Driver Controller
    driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

    auxController.povDown().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.INTAKE));
    auxController.povLeft().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.SUBWOOFER));
    auxController.povUp().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.AMP));
    auxController.povRight().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.YEET));

    auxController.rightBumper().onTrue(new ShootCommand(intakeSubsystem, armSubsystem));
    auxController.leftBumper().whileTrue(new RunIntakeCommand(intakeSubsystem));
  }
  

  // Use this method to pass the autonomous command to the main class
  public Command getAutonomousCommand() {
    return null;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}