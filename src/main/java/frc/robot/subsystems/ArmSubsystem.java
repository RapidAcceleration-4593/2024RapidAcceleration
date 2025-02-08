package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;

public class ArmSubsystem extends SubsystemBase {
    
    private final CANSparkMax leftGearbox1 = ArmConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = ArmConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = ArmConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = ArmConstants.rightGearbox2;

    private final DigitalInput topLimitSwitch = ArmConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ArmConstants.bottomLimitSwitch;

    private final Encoder armEncoder = ArmConstants.primaryNeckEncoder;

    private final PIDController armPIDController = new PIDController(0, 0, 0);

    private final double[] setpoints = {0, 40, 100, 250};

    public ArmSubsystem() {
        // leftGearbox1.setIdleMode(IdleMode.kBrake);
        // leftGearbox2.setIdleMode(IdleMode.kBrake);
        // rightGearbox1.setIdleMode(IdleMode.kBrake);
        // rightGearbox2.setIdleMode(IdleMode.kBrake);
    }

    private double getArmStates(ArmStates state) {
        return switch (state) {
            case INTAKE -> setpoints[0];
            case SUBWOOFER -> setpoints[1];
            case YEET -> setpoints[2];
            case AMP -> setpoints[3];
        };
    }

    public void setArmSetpoint(ArmStates state) {
        armPIDController.setSetpoint(getArmStates(state));
    }

    /** ----- Arm State System ----- */

    // Default Command
    public void maintainArmState() {
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            // System Malfunction.
            stopArmMotors();
        } else if (isTopLimitSwitchPressed()) {
            // Top Limit Switch.
            handleTopLimitSwitchPressed();
        } else if (isBottomLimitSwitchPressed()) {
            // Bottom Limit Switch.
            handleBottomLimitSwitchPressed();
        } else {
            // Regular Control.
            updatePIDConstants(getEncoderValue(), getArmSetpoint(), ArmConstants.PID_THRESHOLD);
        }
        SmartDashboard.putNumber("ArmEncoder", getEncoderValue());
        SmartDashboard.putNumber("Setpoint", getArmSetpoint());
    }

    private void updatePIDConstants(double encoder, double setpoint, double threshold) {
        double error = Math.abs(setpoint - encoder);
        double difference = encoder - setpoint;
        double kP, kI, kD;

        if (error <= 20 && setpoint != setpoints[0] && setpoint != setpoints[3]) {
            // Close Controller
            kP = 0.0155;
            kI = 0.00125;
            kD = 0;
            armPIDController.setPID(kP, kI, kD);
            setArmSpeed((armPIDController.calculate(encoder, setpoint)));
        } else if (difference < 0 && encoder < 200 && setpoint != setpoints[0]) {
            // Up Controller
            kP = 0.0015;
            kI = 0;
            kD = 0;
            armPIDController.setPID(kP, kI, kD);
            setArmSpeed((armPIDController.calculate(encoder, setpoint)));
        } else if (difference > 0 && encoder > 25 && encoder < 350 && setpoint != setpoints[3]) {
            // Down Controller
            kP = 0.0022;
            kI = 0;
            kD = 0;
            armPIDController.setPID(kP, kI, kD);
            setArmSpeed((armPIDController.calculate(encoder, setpoint)));
        }

        // if (!isBottomLimitSwitchPressed() && setpoint == setpoints[0] && encoder <= 25 && encoder > 3) {
        //     setArmSpeed(-0.12);
        // }

        // if (!isTopLimitSwitchPressed()) {
        //     if (setpoint == setpoints[3] && encoder >= 200 && encoder < 290) {
        //         setArmSpeed(0.16);
        //     }
        // }
    }

    private void handleTopLimitSwitchPressed() {
        armPIDController.setSetpoint(getEncoderValue());

        if (getArmSetpoint() < getEncoderValue()) {
            setArmSpeed(armPIDController.calculate(getEncoderValue(), getArmSetpoint()));
        } else {
            stopArmMotors();
        }
    }

    private void handleBottomLimitSwitchPressed() {
        if (getArmSetpoint() > 0) {
            setArmSpeed(armPIDController.calculate(getEncoderValue(), getArmSetpoint()));
        } else {
            stopArmMotors();
            resetArmEncoder();
        }
    }

    private boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    private boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    private void setArmSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    private void stopArmMotors() {
        leftGearbox1.stopMotor();
        leftGearbox2.stopMotor();
        rightGearbox1.stopMotor();
        rightGearbox2.stopMotor();
    }

    private double getEncoderValue() {
        return Math.abs(armEncoder.get() / 2);
    }

    private double getArmSetpoint() {
        return armPIDController.getSetpoint();
    }

    private void resetArmEncoder() {
        armEncoder.reset();
    }
}
