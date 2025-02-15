package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final Timer shooterTimer = new Timer();
    private final Timer intakeTimer = new Timer();

    public ShootCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
        shooterTimer.start();
    }

    @Override
    public void execute() {
        intakeSubsystem.runShooter();

        if (shooterTimer.get() < 0.75F) {
            return;
        }
        intakeSubsystem.startIntake();
        intakeTimer.start();
    }
    
    @Override
    public boolean isFinished() {
        return intakeTimer.get() > 0.4F;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopShooter();
        shooterTimer.stop();
        intakeTimer.stop();
    }
}