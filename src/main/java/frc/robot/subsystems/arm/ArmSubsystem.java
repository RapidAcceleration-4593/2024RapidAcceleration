package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmSubsystem extends SubsystemBase {

    private final ArmInputsAutoLogged inputs;
    private final ArmIO io;

    private static final double[] SETPOINTS = {0, 40, 100, 250};
    private final PIDController controller;

    public ArmSubsystem(ArmIO io) {
        this.io = io;
        this.inputs = new ArmInputsAutoLogged();

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(5.0);

        // Trigger topLimitSwitchTrigger = new Trigger(() -> inputs.topLimitSwitch);
        // Trigger bottomLimitSwitchTrigger = new Trigger(() -> inputs.bottomLimitSwitch);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command goToSetpointCommand(double setpoint) {
        return runOnce(() -> controller.setSetpoint(MathUtil.clamp(setpoint, 0, 250)))
                .andThen(run(() -> {
                    double output = controller.calculate(setpoint);
                    io.setSpeed(output);
                }))
                .until(this::shouldStop)
                .finallyDo(() -> {
                    stop();
                    controller.setSetpoint(inputs.angle);
                });
    }

    private boolean shouldStop() {
        if (inputs.topLimitSwitch || inputs.bottomLimitSwitch) return true;
        return controller.atSetpoint();
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
