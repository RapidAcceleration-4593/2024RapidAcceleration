package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmInputs {
        public double angle = 0.0;
        public boolean topLimitSwitch = false;
        public boolean bottomLimitSwitch = false;
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ArmInputs inputs) {}

    /** Sets the speed of the arm motors. */
    public default void setSpeed(double speed) {}

    /** Stops the arm motors immediately. */
    public default void stop() {}

    /** Resets the arm encoder to zero position. */
    public default void resetEncoder() {}
}
