package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmSubsystem extends SubsystemBase {

    private final ArmInputsAutoLogged inputs;
    private final ArmIO io;

    private final PIDController controller;

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        this.inputs = new ArmInputsAutoLogged();

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(5.0);

        Trigger topLimitSwitchTrigger = new Trigger(() -> inputs.topLimitSwitch);
        topLimitSwitchTrigger.onTrue(Commands.runOnce(() -> {
            if (getTargetSetpoint() >= getCurrentReading()) {
                stop();
                controller.setSetpoint(getCurrentReading());
            }
        }));

        Trigger bottomLimitSwitchTrigger = new Trigger(() -> inputs.bottomLimitSwitch);
        bottomLimitSwitchTrigger.onTrue(Commands.runOnce(() -> {
            io.resetEncoder();
            if (getTargetSetpoint() <= 0) {
                stop();
                controller.setSetpoint(0);
            }
        }));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("ArmEncoder", inputs.angle);
        SmartDashboard.putBoolean("ArmTopLS", inputs.topLimitSwitch);
        SmartDashboard.putBoolean("ArmBotLS", inputs.bottomLimitSwitch);
    }

    public Command goToSetpointCommand(double setpoint) {
        return runOnce(() -> controller.setSetpoint(setpoint))
                .andThen(run(() -> {
                    double output = controller.calculate(getCurrentReading());
                    output = MathUtil.clamp(output, -12.0, 12.0);
                    io.setVoltage(Volts.of(output));
                }))
                .finallyDo(() -> {
                    stop();
                    controller.setSetpoint(inputs.angle);
                });
    }

    public double getCurrentReading() {
        return inputs.angle;
    }

    @AutoLogOutput(key = "Arm/TargetAngle")
    public double getTargetSetpoint() {
        return controller.getSetpoint();
    }

    @AutoLogOutput(key = "Arm/AtTargetAngle")
    public boolean atTargetSetpoint() {
        return controller.atSetpoint();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void stop() {
        controller.reset();
        io.stop();
    }
}
