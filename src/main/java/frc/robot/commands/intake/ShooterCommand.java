package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterCommand extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    
    public ShooterCommand(IntakeSubsystem subsystem) {
        this.intakeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.runShooter();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopShooter();
    }
}
