package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PWMSparkMax shooterTopMotor = IntakeConstants.shooterTopMotor;
    private final PWMSparkMax shooterBottomMotor = IntakeConstants.shooterBottomMotor;

    public IntakeSubsystem() {

    }

    /** ----- Abstraction Methods ----- */


    /** ----- Command Factory Methods ----- */

    public void runShooterCommand() {
        shooterTopMotor.set(1.0);
        shooterBottomMotor.set(1.0);
    }
    
    public void stopShooterCommand() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }
}
