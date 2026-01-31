package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final Timer shooterTimer;
    private final Timer intakeTimer;
    private boolean intakeStarted;

    public ShootCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);

        shooterTimer = new Timer();
        intakeTimer = new Timer();
    }

    @Override
    public void initialize() {
        shooterTimer.reset();
        shooterTimer.start();

        intakeTimer.reset();
        intakeStarted = false;
    }

    @Override
    public void execute() {
        intakeSubsystem.runShooters();

        if (!intakeStarted && shooterTimer.hasElapsed(0.8)) {
            intakeSubsystem.runArmIntake();
            intakeTimer.start();
            intakeStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return intakeStarted && intakeTimer.hasElapsed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakes();
        intakeSubsystem.stopShooters();
        shooterTimer.stop();
        intakeTimer.stop();
    }
}
