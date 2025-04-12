package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ContinuousShootCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public ContinuousShootCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runArmIntake();
        intakeSubsystem.runShooters();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakes();
        intakeSubsystem.stopShooters();
    }
}
