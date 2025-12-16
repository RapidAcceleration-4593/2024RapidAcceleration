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

    /** Runs intake motors if limit switch isn't pressed. */
    public void runIntakes() {
        if (!intakeLimitSwitch.get()) {
            setIntakes(1.0);
        } else {
            stopIntakes();
        }
    }

    /**
     * Sets speed of intake motors.
     * @param speed speed of intake motors.
     */
    public void setIntakes(Double speed) {
        bumperIntakeMotor.set(speed);
        armIntakeMotor.set(-speed);
    }

    /** Stops intake motors. */
    public void stopIntakes() {
        armIntakeMotor.stopMotor();
        bumperIntakeMotor.stopMotor();
    }

    /** Runs shooter motors and spins intakes to push nodes into shooter. */
    public void runShooter() {
        topShooterMoter.set(1.0);
        bottomShooterMoter.set(1.0);
        setIntakes(1.0);
    }

    /** Stops shooter motors. */
    public void stopShooter() {
        topShooterMoter.stopMotor();
        bottomShooterMoter.stopMotor();
        stopIntakes();
    }
}
