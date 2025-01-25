package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    // Define Empty CANSparkMax Motor Controller Object
    private final PWMSparkMax climberMotor = ClimberConstants.climberMotor;;

    public void ClimberUp() {
        // Run Climber Up when command is called
        climberMotor.set(1.0);
    }

    public void ClimberDown() {
        // Run Climber Down when command is called
        climberMotor.set(-1.0);
    }

    public void ClimberStop() {
        // When no other climber commands are called, stop driving the motor
        climberMotor.set(0.0);
    }
}
