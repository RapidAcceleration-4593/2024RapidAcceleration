package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZoomModeCommand extends Command {

    private final SwerveSubsystem subsystem;

    public ZoomModeCommand(SwerveSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        subsystem.zoomMode = true;
        subsystem.turnboMode = true;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.zoomMode = false;
        subsystem.turnboMode = false;
    }
}
