package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final Encoder heightEncoder = NeckConstants.primaryNeckEncoder;

    // when set to -1, elevator goes down. when set to 1, elevator goes up.
    private final CANSparkMax elevatorMotor = ClimberConstants.climberMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private static final double[] setpoints = {0, 0, 6000, 23000};

    public ElevatorSubsystem() {
        elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    private double getElevatorLevelSetpoint(ElevatorConstants.ElevatorLevel level) {
        switch (level){
            case L1:
            return setpoints[0];
            case L2:
            return setpoints[1];
            case L3:
            return setpoints[2];
            case L4:
            return setpoints[3];
            default:
            return -1;
        }
    }

    public void raiseElevatorToLevel(ElevatorConstants.ElevatorLevel level) {
        elevatorPID.setSetpoint(getElevatorLevelSetpoint(level));
    }

    /**@return The height encoder, with increasing elevator height being an encoder value closer to positive infinity. */
    private double readEncoderNormalized() {
        return heightEncoder.get() * -1;
    }

    public void moveElevatorUp() {
        elevatorPID.setSetpoint(elevatorPID.getSetpoint() + ElevatorConstants.elevatorManualMovementSpeed);
    }

    public void moveElevatorDown() {
        elevatorPID.setSetpoint(elevatorPID.getSetpoint() - ElevatorConstants.elevatorManualMovementSpeed);
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**ATTENTION!<p>
     * This method (and handleTopLSPressed and handleBottomLSPressed) assumes that moving the elevator up is done by
     * setting the motor speed to a positive value,
     * and assumes that a higher encoder value means that the elevator is higher.
     * This method executes the PID function, while taking into account limit switches.
     * It also resets the encoder when the elevator hits the bottom limit switch.
     */

    public void runPID() {

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            elevatorMotor.set(0);
            return;
        }

        if (isTopLimitSwitchPressed()) {
            handleTopLSPressed();
        }
        else if (isBottomLimitSwitchPressed()) {
            handleBottomLSPressed();
        }
        else {
            elevatorMotor.set(elevatorPID.calculate(readEncoderNormalized())); 
        }
    }

    private void handleTopLSPressed() {
        if (elevatorPID.getSetpoint() > readEncoderNormalized()) {
            elevatorPID.setSetpoint(readEncoderNormalized());
        }

        // NOTE!!!! This if statement assumes that moving the elevator up is done by setting the motor speed to a positive value
        var pidOutput = elevatorPID.calculate(readEncoderNormalized());

        if (pidOutput > 0.005) {
            elevatorMotor.set(0);
        }
        else {
            System.out.println("here");
            elevatorMotor.set(pidOutput);
        }
    }

    private void handleBottomLSPressed() {
        heightEncoder.reset();

        if (elevatorPID.getSetpoint() < readEncoderNormalized()) {
            elevatorPID.setSetpoint(readEncoderNormalized());
        }

        // NOTE!!!! This if statement assumes that moving the elevator down is done by setting the motor speed to a negative value
        var pidOutput = elevatorPID.calculate(readEncoderNormalized());

        if (pidOutput < 0.005) {
            elevatorMotor.set(0);
        }
        else {
            elevatorMotor.set(pidOutput);
        }
    }

    // Factory Command Section: Add all command factory methods beneath this line.

    public Command moveElevatorUpCommand() {
        return runOnce(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand() {
        return runOnce(() -> moveElevatorDown());
    }
}