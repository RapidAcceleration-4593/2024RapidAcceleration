package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final ArmSubsystem armSubsystem;
    private final Timer shooterTimer;
    private final Timer intakeTimer;

    public ShootCommand(IntakeSubsystem intake, ArmSubsystem arm) {
        this.intakeSubsystem = intake;
        this.armSubsystem = arm;
        addRequirements(arm, intake);
        
        shooterTimer = new Timer();
        intakeTimer = new Timer();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public void initialize() {
        intakeTimer.stop();
        intakeTimer.reset();

        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimer.start();
    }

    @Override
    public void execute() {
        armSubsystem.maintainArmState();
        intakeSubsystem.runShooter();

        if (shooterTimer.hasElapsed(1.6)) {
            if (intakeTimer.get() == 0) {
                intakeSubsystem.runArmIntake();
                intakeTimer.start();
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return intakeTimer.hasElapsed(1.2);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopShooter();
        shooterTimer.stop();
        intakeTimer.stop();
    }
}