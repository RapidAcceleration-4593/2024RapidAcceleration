package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {
    
    private final SparkMax leftGearbox1 = ArmConstants.leftGearbox1;
    private final SparkMax leftGearbox2 = ArmConstants.leftGearbox2;
    private final SparkMax rightGearbox1 = ArmConstants.rightGearbox1;
    private final SparkMax rightGearbox2 = ArmConstants.rightGearbox2;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;

    private final Encoder armEncoder = ArmConstants.primaryNeckEncoder;

    private final PIDController armPIDController = new PIDController(ArmConstants.ARM_PID.kP, ArmConstants.ARM_PID.kI, ArmConstants.ARM_PID.kD);

    private static final double[] SETPOINTS = {0, 40, 100, 250};

    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightConfig = new SparkMaxConfig();

    public ArmSubsystem() {
        leaderConfig.idleMode(IdleMode.kBrake);
        leftConfig.idleMode(IdleMode.kBrake).follow(leftGearbox1, false);
        rightConfig.idleMode(IdleMode.kBrake).follow(leftGearbox1, true);

        leftGearbox1.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftGearbox2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox1.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox2.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armPIDController.setTolerance(5);
    }


    /** ----- Arm State Management ----- */

    /**
     * Retrieves the setpoint from the specified arm states.
     * @param state The desired arm position.
     * @return The {@link ArmSubsystem#SETPOINTS} value corresponding to the state.
     */
    private double getArmState(ArmStates state) {
        return switch (state) {
            case INTAKE -> SETPOINTS[0];
            case SUBWOOFER -> SETPOINTS[1];
            case YEET -> SETPOINTS[2];
            case AMP -> SETPOINTS[3];
            default -> throw new Error("Passed in an ArmState that does not have an associated setpoint!");
        };
    }

    /**
     * Sets the setpoint of the arm to the target state.
     * @param state The desired arm position.
     */
    public void setArmState(ArmStates state) {
        setSetpoint(getArmState(state));
    }


    /** ----- Arm State System ----- */

    /**
     * Controls arm movement based on limit switch inputs and encoder feedback.
     * <ul>
     *  <li>Stops the motor if both the top and bottom limit switches are pressed (i.e. system malfunction).</li>
     *  <li>Handles specific behavior when either the top or bottom limit switch is pressed.</li>
     *  <li>Uses PID Control to adjust the motor output when no limit switches are triggered.</li>
     * </ul>
     */
    public void maintainArmState() {
        updateValues();

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            stopMotors();
        } else if (isTopLimitSwitchPressed()) {
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            handleBottomLimitSwitchPressed();
        } else {
            controlArm();
        }
    }

    /** Controls the Arm System using a PID Controller. */
    private void controlArm() {
        double output = armPIDController.calculate(getEncoderValue(), getSetpoint());

        if (atSetpoint()) {
            stopMotors();
        } else {
            setMotorSpeeds(output);
        }
    }


    /** ----- Limit Switch Handling ----- */

    /**
     * Handles the behavior when the top limit switch is pressed.
     * <ul>
     *  <li>Stops the motor and sets the current position as the new setpoint.</li>
     *  <li>Allows downward movement without interference.</li>
     * </ul>
     */
    private void handleTopLimitSwitchPressed() {
        if (getSetpoint() >= getEncoderValue()) {
            stopMotors();
            setSetpoint(getEncoderValue());
        } else {
            controlArm();
        }
    }

    /**
     * Handles the behavior when the bottom limit switch is pressed.
     * <ul>
     *  <li>Resets the encoder, stops the motor, and sets the current position as the new setpoint. </li>
     *  <li>Allows upward movement without interference.</li>
     * </ul>
     */
    private void handleBottomLimitSwitchPressed() {
        resetEncoder();

        if (getSetpoint() <= 0) {
            stopMotors();
            setSetpoint(0);
        } else {
            controlArm();
        }
    }


    /** ----- Motor, Encoder, and Limit Switch Abstraction ----- */

    /**
     * Checks if the top limit switch is pressed.
     * @return Whether {@link ArmSubsystem#topLimitSwitch} is pressed.
     */
    private boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    /**
     * Checks if the bottom limit switch is pressed.
     * @return Whether {@link ArmSubsystem#bottomLimitSwitch} is pressed.
     */
    private boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**
     * Sets the motor speed for the arm motors.
     * @param speed The desired speed of the motors.
     */
    private void setMotorSpeeds(double speed) {
        leftGearbox1.set(speed);
    }

    /** Stops movement for the arm motors. */
    private void stopMotors() {
        leftGearbox1.stopMotor();
    }

    /**
     * Retrieves the current encoder value of the arm.
     * @return The current encoder value of the arm.
     */
    private double getEncoderValue() {
        return Math.abs(armEncoder.get() / 2); // TODO: Adjust for higher resolution and negative numbers.
    }

    /** Resets the arm encoder. */
    private void resetEncoder() {
        armEncoder.reset();
    }

    /**
     * Gets the setpoint for the Arm PID Controller.
     * @return The current numerical setpoint value.
     */
    private double getSetpoint() {
        return armPIDController.getSetpoint();
    }

    /**
     * Whether the arm is at its setpoint.
     * @return If the arm is at the setpoint, accounting for tolerance.
     */
    private boolean atSetpoint() {
        return armPIDController.atSetpoint();
    }

    /**
     * Sets the setpoint for the arm PID Controller, without resetting.
     * @param setpoint New setpoint value.
     */
    public void setSetpoint(double setpoint) {
        armPIDController.setSetpoint(setpoint);
    }

    /** Updates values to SmartDashboard/ShuffleBoard. */
    private void updateValues() {
        SmartDashboard.putNumber("Encoder", getEncoderValue());
        SmartDashboard.putNumber("Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Top-LS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("Bot-LS", isBottomLimitSwitchPressed());
    }
}
