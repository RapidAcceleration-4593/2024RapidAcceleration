package frc.robot;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    public RobotContainer() {
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        arm = new ArmSubsystem(new ArmIOReal());
        intake = new IntakeSubsystem();

        controller = new CommandXboxController(kControllerPort);

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(driveCommand());

        controller.back().onTrue(Commands.runOnce(swerve::zeroGyro));

        // controller.povDown().onTrue(new SetArmState(arm, ArmStates.INTAKE));
        // controller.povLeft().onTrue(new SetArmState(arm, ArmStates.SUBWOOFER));
        // controller.povRight().onTrue(new SetArmState(arm, ArmStates.YEET));
        // controller.povUp().onTrue(new SetArmState(arm, ArmStates.AMP));

        controller.leftBumper().whileTrue(new IntakeCommand(intake));
        controller.rightBumper().onTrue(new ShootCommand(intake));

        controller.x().whileTrue(new ContinuousShootCommand(intake));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }

    private Command driveCommand() {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        swerve.getSwerveDrive(), () -> -controller.getLeftY(), () -> -controller.getLeftX())
                .withControllerRotationAxis(() -> -controller.getRightX())
                .deadband(kDeadband)
                .scaleTranslation(kTranslationScale)
                .allianceRelativeControl(true);

        return swerve.driveFieldOriented(driveAngularVelocity);
    }
}
