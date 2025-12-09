package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PWMSparkMax bumperIntakeMotor = IntakeConstants.bumperIntakeMotor;
    private final PWMSparkMax armIntakeMotor = IntakeConstants.armIntakeMotor;
    private final PWMSparkMax topShooterMoter = IntakeConstants.topShooterMotor;
    private final PWMSparkMax bottomShooterMoter = IntakeConstants.bottomShooterMotor;

    private final DigitalInput intakeLimitSwitch = IntakeConstants.intakeLimitSwitch;

    public IntakeSubsystem() {
        // Constructor
    }

    public void runIntakes() {
        if (!intakeLimitSwitch.get()) {
            bumperIntakeMotor.set(1.0);
            armIntakeMotor.set(-1.0);
        } else {
            stopIntakes();
        }
    }

    public void stopIntakes() {
        armIntakeMotor.stopMotor();
        bumperIntakeMotor.stopMotor();
    }

    public void runShooter() {
        topShooterMoter.set(1.0);
        bottomShooterMoter.set(1.0);
    }

    public void stopShooter() {
        topShooterMoter.stopMotor();
        bottomShooterMoter.stopMotor();
    }
}
