package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ZoomModeCommand;
import frc.robot.commands.intake.ContinuousShootCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ShootCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final ArmSubsystem arm;
    public final IntakeSubsystem intake;

    // Controller(s)
    private final CommandXboxController controller;
    private final CommandXboxController secondaryController;

    public RobotContainer() {
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        arm = new ArmSubsystem(new ArmIOReal());
        intake = new IntakeSubsystem();

        controller = new CommandXboxController(kControllerPort);
        secondaryController = new CommandXboxController(1);

        configureBindings();
    }

    private void configureBindings() {
        controller.a().whileTrue(new ZoomModeCommand(swerve));

        swerve.setDefaultCommand(driveCommand());

        controller.back().onTrue(Commands.runOnce(swerve::zeroGyro));
        secondaryController.back().onTrue(Commands.runOnce(swerve::zeroGyro));

        controller.povDown().onTrue(arm.goToSetpointCommand(0));
        controller.povLeft().onTrue(arm.goToSetpointCommand(40));
        controller.povRight().onTrue(arm.goToSetpointCommand(100));
        controller.povUp().onTrue(arm.goToSetpointCommand(250));

        secondaryController.povDown().onTrue(arm.goToSetpointCommand(0));
        secondaryController.povLeft().onTrue(arm.goToSetpointCommand(40));
        secondaryController.povRight().onTrue(arm.goToSetpointCommand(100));
        secondaryController.povUp().onTrue(arm.goToSetpointCommand(250));

        controller.leftBumper().whileTrue(new IntakeCommand(intake));
        controller.rightBumper().onTrue(new ShootCommand(intake));

        secondaryController.leftBumper().whileTrue(new IntakeCommand(intake));
        secondaryController.rightBumper().onTrue(new ShootCommand(intake));

        controller.x().whileTrue(new ContinuousShootCommand(intake));
        secondaryController.x().whileTrue(new ContinuousShootCommand(intake));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }

    private Command driveCommand() {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        swerve.getSwerveDrive(),
                        () -> -controller.getLeftY() * (swerve.zoomMode ? 1.0 : 0.4),
                        () -> -controller.getLeftX() * (swerve.zoomMode ? 1.0 : 0.4))
                .withControllerRotationAxis(() -> -controller.getRightX() / (swerve.turnboMode ? 1.0 : 2.0))
                .deadband(kDeadband)
                .scaleTranslation(kTranslationScale)
                .allianceRelativeControl(true);

        return swerve.driveFieldOriented(driveAngularVelocity);
    }
}
