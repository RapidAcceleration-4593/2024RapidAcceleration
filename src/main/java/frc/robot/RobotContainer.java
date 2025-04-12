// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.MaintainArmState;
import frc.robot.commands.arm.SetArmSetpoint;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.intake.ShootCommand;
import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;

import java.io.File;

/**
 * This class defines the robot's structure, including subsystems, commands, and trigger mappings.
 * Most robot logic is managed here, not in the {@link Robot} periodic methods.
 */
public class RobotContainer {
    // Subsystem(s)
    public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    /** Converts driver input into a field-relative ChassisSpeeds that is controller by angular velocity. */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX())
                                                                .withControllerRotationAxis(() -> -driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
                                                                .allianceRelativeControl(true);

    /** Clones the angular velocity input stream and converts it to a robotRelative input stream. */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                                    .allianceRelativeControl(false);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    public RobotContainer() {
        configureBindings();

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        armSubsystem.setDefaultCommand(new MaintainArmState(armSubsystem));
    }

    private void configureBindings() {
        driverController.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        driverController.povDown().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.INTAKE));
        driverController.povLeft().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.SUBWOOFER));
        driverController.povUp().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.AMP));
        driverController.povRight().onTrue(new SetArmSetpoint(armSubsystem, ArmStates.YEET));

        driverController.rightBumper().onTrue(new ShootCommand(intakeSubsystem, armSubsystem));
        driverController.leftBumper().whileTrue(new RunIntakeCommand(intakeSubsystem));
    }
    

    // Use this method to pass the autonomous command to the main class
    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}