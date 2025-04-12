package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PWMSparkMax topShooterMotor = IntakeConstants.topShooterMotor;
    private final PWMSparkMax bottomShooterMotor = IntakeConstants.bottomShooterMotor;

    private final PWMSparkMax bumperIntakeMotor = IntakeConstants.bumperIntakeMotor;
    private final PWMSparkMax armIntakeMotor = IntakeConstants.armIntakeMotor;

    private final DigitalInput intakeLimitSwitch = IntakeConstants.intakeLimitSwitch;

    public IntakeSubsystem() {
        // Constructor
    }

    /**
     * Sets the speed of the shooter motors.
     * @param speed The speed to set the shooter motors to, between -1.0 and 1.0.
     */
    private void setShooterSpeed(double speed) {
        topShooterMotor.set(speed);
        bottomShooterMotor.set(speed);
    }

    /**
     * Sets the speed of the intake motors.
     * @param speed The speed to set the intake motors to, between -1.0 and 1.0.
     */
    private void setIntakeSpeeds(double speed) {
        bumperIntakeMotor.set(speed);
        armIntakeMotor.set(-speed);
    }
    
    /** Runs the intake motors if the limit switch is not pressed. */
    public void runIntakes() {
        if (!intakeLimitSwitch.get()) {
            setIntakeSpeeds(1.0);
        } else {
            stopIntakes();
        }
    }

    /** Runs the intake on the Arm. */
    public void runArmIntake() {
        armIntakeMotor.set(-1.0);
    }

    /** Stops the intake motors. */
    public void stopIntakes() {
        bumperIntakeMotor.stopMotor();
        armIntakeMotor.stopMotor();
    }

    /** Runs both shooters. */
    public void runShooters() {
        setShooterSpeed(1.0);
    }
    
    /** Stops both shooters. */
    public void stopShooters() {
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }
}
