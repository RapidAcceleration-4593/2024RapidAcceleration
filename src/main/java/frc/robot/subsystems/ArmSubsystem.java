package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {
    
    private final SparkMax leftGearbox1;
    private final SparkMax leftGearbox2;
    private final SparkMax rightGearbox1;
    private final SparkMax rightGearbox2;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private final Encoder encoder;
    private final PIDController controller;

    private static final double[] SETPOINTS = {0, 40, 100, 250};

    public ArmSubsystem() {
        leftGearbox1 = new SparkMax(kLeftGearbox1, MotorType.kBrushless);
        leftGearbox2 = new SparkMax(kLeftGearbox2, MotorType.kBrushless);
        rightGearbox1 = new SparkMax(kRightGearbox1, MotorType.kBrushless);
        rightGearbox2 = new SparkMax(kRightGearbox2, MotorType.kBrushless);

        topLimitSwitch = new DigitalInput(kTopLimitSwitchChannel);
        bottomLimitSwitch = new DigitalInput(kBottomLimitSwitchChannel);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(5);

        SparkBaseConfig leaderConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake);
        SparkBaseConfig leftConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake).follow(leftGearbox1, false);
        SparkBaseConfig rightConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake).follow(leftGearbox1, true);

        leftGearbox1.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftGearbox2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox1.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox2.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        double output = controller.calculate(getEncoderValue(), getSetpoint());

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
        return Math.abs(encoder.get() / 2); // TODO: Adjust for higher resolution and negative numbers.
    }

    /** Resets the arm encoder. */
    private void resetEncoder() {
        encoder.reset();
    }

    /**
     * Gets the setpoint for the Arm PID Controller.
     * @return The current numerical setpoint value.
     */
    private double getSetpoint() {
        return controller.getSetpoint();
    }

    /**
     * Whether the arm is at its setpoint.
     * @return If the arm is at the setpoint, accounting for tolerance.
     */
    private boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Sets the setpoint for the arm PID Controller, without resetting.
     * @param setpoint New setpoint value.
     */
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    /** Updates values to SmartDashboard/ShuffleBoard. */
    private void updateValues() {
        SmartDashboard.putNumber("Encoder", getEncoderValue());
        SmartDashboard.putNumber("Setpoint", getSetpoint());
        SmartDashboard.putBoolean("Top-LS", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("Bot-LS", isBottomLimitSwitchPressed());
    }
}
