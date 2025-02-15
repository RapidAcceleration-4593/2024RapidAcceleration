package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    private final PWMSparkMax shooterTopMotor = IntakeConstants.shooterTopMotor;
    private final PWMSparkMax shooterBottomMotor = IntakeConstants.shooterBottomMotor;

    private final PWMSparkMax bumperIntakeMotor = IntakeConstants.bumperIntakeMotor;
    private final PWMSparkMax beakIntakeMotor = IntakeConstants.beakIntakeMotor;

    private final DigitalInput intakeLimitSwitch = IntakeConstants.intakeLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;

    private void setShooterSpeed(double speed) {
        shooterTopMotor.set(speed);
        shooterBottomMotor.set(speed);
    }

    private void setIntakeSpeed(double speed) {
        bumperIntakeMotor.set(speed);
        beakIntakeMotor.set(-speed);
    }
    
    public void intake() {
        if (!bottomLimitSwitch.get() && !intakeLimitSwitch.get()) {
            setIntakeSpeed(1);
        } else {
            stopIntake();
        }
    }

    public void runArmIntake() {
        beakIntakeMotor.set(-1);
    }

    public void outtake() {
        setIntakeSpeed(-1);
    }

    public void stopIntake() {
        setIntakeSpeed(0);
    }

    public void runShooter() {
        setShooterSpeed(.3);
    }
    
    public void stopShooter() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }

}
