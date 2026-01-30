package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class ArmIOReal implements ArmIO {

    protected final SparkMax leftGearbox1;
    protected final SparkMax leftGearbox2;
    protected final SparkMax rightGearbox1;
    protected final SparkMax rightGearbox2;

    protected final DigitalInput topLimitSwitch;
    protected final DigitalInput bottomLimitSwitch;

    protected final Encoder encoder;

    public ArmIOReal() {
        leftGearbox1 = new SparkMax(kLeftGearbox1, MotorType.kBrushed);
        leftGearbox2 = new SparkMax(kLeftGearbox2, MotorType.kBrushed);
        rightGearbox1 = new SparkMax(kRightGearbox1, MotorType.kBrushed);
        rightGearbox2 = new SparkMax(kRightGearbox2, MotorType.kBrushed);

        SparkBaseConfig leaderConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig leftConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(leftGearbox1, false);
        SparkBaseConfig rightConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(leftGearbox1, true);

        leftGearbox1.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftGearbox2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox1.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightGearbox2.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        topLimitSwitch = new DigitalInput(kTopLimitSwitchChannel);
        bottomLimitSwitch = new DigitalInput(kBottomLimitSwitchChannel);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.angle = Math.abs(encoder.get() / 2);
        inputs.topLimitSwitch = !topLimitSwitch.get();
        inputs.bottomLimitSwitch = !bottomLimitSwitch.get();
    }

    @Override
    public void setSpeed(double speed) {
        leftGearbox1.set(speed);
    }

    @Override
    public void stop() {
        leftGearbox1.stopMotor();
    }

    @Override
    public void resetEncoder() {
        encoder.reset();
    }
}
