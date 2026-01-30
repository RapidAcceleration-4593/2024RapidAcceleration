// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.MaintainArmState;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.intake.ContinuousShootCommand;
import frc.robot.commands.intake.IntakeCommand;
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
    public final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ArmSubsystem arm = new ArmSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    // Controller(s)
    private final CommandXboxController controller = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    /** Converts driver input into a field-relative ChassisSpeeds that is controller by angular velocity. */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> -controller.getLeftY(),
                                                                () -> -controller.getLeftX())
                                                                .withControllerRotationAxis(() -> -controller.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
                                                                .allianceRelativeControl(true);

    Command driveFieldOrientedAngularVelocity = swerve.driveFieldOriented(driveAngularVelocity);

    public RobotContainer() {
        configureBindings();

        swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);
        arm.setDefaultCommand(new MaintainArmState(arm));

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        controller.back().onTrue(Commands.runOnce(swerve::zeroGyro));

        controller.povDown().onTrue(new SetArmState(arm, ArmStates.INTAKE));
        controller.povLeft().onTrue(new SetArmState(arm, ArmStates.SUBWOOFER));
        controller.povRight().onTrue(new SetArmState(arm, ArmStates.YEET));
        controller.povUp().onTrue(new SetArmState(arm, ArmStates.AMP));

        controller.leftBumper().whileTrue(new IntakeCommand(intake));
        controller.rightBumper().onTrue(new ShootCommand(intake));

        controller.x().whileTrue(new ContinuousShootCommand(intake));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}