package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunShooterCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;

    public RunShooterCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.runShooterCommand();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooterCommand();
    }
}
