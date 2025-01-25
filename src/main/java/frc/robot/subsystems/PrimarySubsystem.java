package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class PrimarySubsystem extends SubsystemBase {

    private final CANSparkMax leftGearbox1 = NeckConstants.leftGearbox1;
    private final CANSparkMax leftGearbox2 = NeckConstants.leftGearbox2;
    private final CANSparkMax rightGearbox1 = NeckConstants.rightGearbox1;
    private final CANSparkMax rightGearbox2 = NeckConstants.rightGearbox2;

    private final PWMSparkMax bumperIntakeMotor = BeakConstants.bumperIntakeMotor;
    private final PWMSparkMax beakIntakeMotor = BeakConstants.beakIntakeMotor;
    private final PWMSparkMax shooterTopMotor = BeakConstants.shooterTopMotor;
    private final PWMSparkMax shooterBottomMotor = BeakConstants.shooterBottomMotor;

    private final DigitalInput intakeLimitSwitch = BeakConstants.intakeLimitSwitch;
    private final DigitalInput topLimitSwitch = NeckConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = NeckConstants.bottomLimitSwitch;

    private final Encoder primaryNeckEncoder = NeckConstants.primaryNeckEncoder;
    private final Encoder secondaryNeckEncoder = NeckConstants.secondaryNeckEncoder;

    private int primaryNeckEncoderValue = primaryNeckEncoder.get() / -2;

    private enum NeckStates { INTAKE, SUBWOOFER, VISION, YEET, AMP, MANUAL_UP, MANUAL_DOWN, MANUAL_STOP, TEST }
    private NeckStates currentNeckState = NeckStates.INTAKE;

    private final PIDController neckRotateController = new PIDController(0.0, 0.0, 0.0);

    PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

    private boolean manualControlEnabled = false;
    private boolean shooterTimerStarted = false;
    private boolean intakeIntaked = false;
    private boolean subwooferPositionSet = false;
    private boolean shotFirstNote = false;
    private boolean downTimerStarted = false;
    private boolean autoTimerStarted = false;
    private boolean encoderFailureDetected = false;

    private Timer shooterTimer = new Timer();
    private Timer downTimer = new Timer();
    private Timer autoTimer = new Timer();

    private final Spark siccLEDS = LEDConstants.SiccLEDs;

    private int neckGoalAngle = 0;

    public void IntakePosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.INTAKE; subwooferPositionSet = false; }}
    public void SubwooferPosition() { if (!manualControlEnabled) { currentNeckState = NeckStates.SUBWOOFER; }}
    public void YeetPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.YEET; }}
    public void AmpPosition() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.AMP; }}

    public void VisionNeckAnglePressed() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.VISION; }}
    public void VisionNeckAngleNotPressed() { if (!manualControlEnabled && currentNeckState == NeckStates.VISION) { currentNeckState = NeckStates.SUBWOOFER; }}
    public void VisionNeckAngleAuto() { if (!manualControlEnabled && currentNeckState != NeckStates.INTAKE) { currentNeckState = NeckStates.VISION; }}

    public void NeckUp() { if (manualControlEnabled) { if (topLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_UP; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckDown() { if (manualControlEnabled) { if (bottomLimitSwitch.get()) { currentNeckState = NeckStates.MANUAL_DOWN; } else { currentNeckState = NeckStates.MANUAL_STOP; }}}
    public void NeckStop() { if (manualControlEnabled) { currentNeckState = NeckStates.MANUAL_STOP; }}

    public void EnableManualControl() { manualControlEnabled = true; }
    public void DisableManualControl() { if (!bottomLimitSwitch.get() && Math.abs(primaryNeckEncoderValue - secondaryNeckEncoder.get()) < 5) { manualControlEnabled = false; IntakePosition(); }}

    public static String autoName = SmartDashboard.getString("AutoSelector", "DoNothing");
    private CANBusStatus CANInfo = CANBus.getStatus("rio");

    public void periodic() {
        primaryNeckEncoderValue = primaryNeckEncoder.get() / -2;
        
        camera.setLED(VisionLEDMode.kOff);

        SmartDashboard.putNumber("MatchTimeRemaining", Math.round(DriverStation.getMatchTime()));
        SmartDashboard.putNumber("CANBusUtilization", Math.round(CANInfo.BusUtilization * 100));

        SmartDashboard.putBoolean("DriverControllerConnected", DriverStation.isJoystickConnected(0));
        SmartDashboard.putBoolean("AuxControllerConnected", DriverStation.isJoystickConnected(1));
        SmartDashboard.putBoolean("EncoderFailureDetected", encoderFailureDetected);
        SmartDashboard.putBoolean("ManualControlEnabled", manualControlEnabled);
        
        SmartDashboard.putString("ArmState", currentNeckState.name());
        SmartDashboard.putNumber("PrimaryEncoderValue", primaryNeckEncoderValue);
        SmartDashboard.putNumber("SecondaryEncoderValue", secondaryNeckEncoder.get());
        SmartDashboard.putBoolean("BottomLimitSwitchValue", !bottomLimitSwitch.get());
        SmartDashboard.putBoolean("TopLimitSwitchValue", !topLimitSwitch.get());
        SmartDashboard.putBoolean("IntakeLimitSwitchValue", intakeLimitSwitch.get());

        autoName = SmartDashboard.getString("AutoSelector", "DoNothing");

        if (!autoTimerStarted && primaryNeckEncoderValue > 10) {
            autoTimer.start();
            autoTimerStarted = true;
        } else if (autoTimerStarted && autoTimer.get() >= 15) {
            autoTimer.stop();
            if (!shooterTimerStarted && shotFirstNote) {
                shooterTopMotor.set(0.15);
                shooterBottomMotor.set(0.15);
            }
        }

        ManageNeckStates();
        UpdatePIDConstants();
        ManageEncoderFailure();

        // System.out.println("<--------------->");
        // System.out.println("Goal: " + neckGoalAngle);
        // System.out.println("Primary Encoder Value: " + primaryNeckEncoderValue);
        // System.out.println("Secondary Encoder Value: " + secondaryNeckEncoder.get());

        // System.out.println("Top Limit Switch: " + topLimitSwitch.get());
        // System.out.println("Bottom Limit Switch: " + bottomLimitSwitch.get());
        // System.out.println("Intake Limit Switch: " + intakeLimitSwitch.get());

        // System.out.println("Manual Control Enabled: " + manualControlEnabled);
        // System.out.println("Neck State: " + currentNeckState);
    }

    private void ManageNeckStates() {
        switch (currentNeckState) {
            case INTAKE:
                neckGoalAngle = 0;
                break;
            case SUBWOOFER:
                neckGoalAngle = 40;
                break;
            case VISION:
                if (camera.getLatestResult().hasTargets()) {
                    List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
                    for (PhotonTrackedTarget target : targets) {
                        int targetID = target.getFiducialId();
                        if (targetID == 4 || targetID == 7) {
                            double distanceX = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
                            double distanceY = camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
                            double distanceHypo = Math.hypot(distanceX, distanceY);
                            neckGoalAngle = VisionSetAngle(distanceHypo);
                        }
                    }
                } else {
                    System.out.println("No Targets Detected");
                } 
                break;
            case YEET:
                neckGoalAngle = 100;
                break;
            case AMP:
                neckGoalAngle = 250;
                break;
            case MANUAL_UP:
                NeckSetRotateSpeed(0.16);
                break;
            case MANUAL_DOWN:
                NeckSetRotateSpeed(-0.07);
                break;
            case MANUAL_STOP:
                NeckSetRotateSpeed(0.0);
                break;
            default:
                break;
        }
    }

    private void UpdatePIDConstants() {
        if (!manualControlEnabled) {
            double p, i, d;
            double error = primaryNeckEncoderValue - neckGoalAngle;

            if (Math.abs(error) <= 20 && currentNeckState != NeckStates.INTAKE && currentNeckState != NeckStates.AMP) {
                // Close Controller
                p = 0.0155;
                i = 0.00125;
                d = 0.0;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoderValue, neckGoalAngle));
            } else if (error < 0 && primaryNeckEncoderValue < 200 && currentNeckState != NeckStates.INTAKE) {
                // Up Controller
                p = 0.0015;
                i = 0.0075;
                d = 0.0;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoderValue, neckGoalAngle));
            } else if (error > 0 && primaryNeckEncoderValue > 25 && primaryNeckEncoderValue < 350 && currentNeckState != NeckStates.AMP) {
                // Down Controller
                p = 0.0022;
                i = 0.0;
                d = 0.0;
                neckRotateController.setPID(p, i, d);
                NeckSetRotateSpeed(neckRotateController.calculate(primaryNeckEncoderValue, neckGoalAngle));
            }

            ManageLimitSwitches();
        } else {
            siccLEDS.set(0.87);
        }

        if (!bottomLimitSwitch.get() && currentNeckState == NeckStates.INTAKE) {
            primaryNeckEncoder.reset();
            secondaryNeckEncoder.reset();
            NeckSetRotateSpeed(0.0);
        }
    }

    private void ManageLimitSwitches() {
        if (bottomLimitSwitch.get()) {
            if (currentNeckState == NeckStates.INTAKE && primaryNeckEncoderValue <= 25 && primaryNeckEncoderValue > 3) {
                NeckSetRotateSpeed(-0.12);
            }
        }

        if (topLimitSwitch.get()) {
            if (currentNeckState == NeckStates.AMP && primaryNeckEncoderValue >= 200 && primaryNeckEncoderValue < 290) {
                NeckSetRotateSpeed(0.16);
            }
        } else if (!topLimitSwitch.get() && currentNeckState == NeckStates.AMP) {
            NeckSetRotateSpeed(0.0);
        }

        if (intakeLimitSwitch.get()) {
            siccLEDS.set(0.77); // GREEN SOLID
            if (!intakeIntaked) {
                IntakeStop();
                intakeIntaked = true;
            }

            if (!bottomLimitSwitch.get() && shotFirstNote && !subwooferPositionSet && currentNeckState == NeckStates.INTAKE) {
                SubwooferPosition();
                subwooferPositionSet = true;
            }
        } else {
            siccLEDS.set(0.61); // RED SOLID
            intakeIntaked = false;
            shotFirstNote = true;

            if (bottomLimitSwitch.get() && subwooferPositionSet && currentNeckState != NeckStates.INTAKE) {
                if (!downTimerStarted) {
                    downTimer.start();
                    downTimerStarted = true;
                }
            }
        }

        if (downTimerStarted) {
            if (downTimer.get() > 0.0 && downTimer.get() < 0.25) {
                IntakeCheck();
            } else if (downTimer.get() > 0.25) {
                IntakePosition();
                subwooferPositionSet = false;
                downTimerStarted = false;
                downTimer.stop();
                downTimer.reset();
            }
        }
    }

    private void ManageEncoderFailure() {
        double encoderDifference = Math.abs(primaryNeckEncoderValue - secondaryNeckEncoder.get());
        if (!manualControlEnabled && (encoderDifference > 20)) {
            manualControlEnabled = true;
            encoderFailureDetected = true;
        }
    }

    private void NeckSetRotateSpeed(double speed) {
        leftGearbox1.set(speed);
        leftGearbox2.follow(leftGearbox1, false);
        rightGearbox1.follow(leftGearbox1, true);
        rightGearbox2.follow(leftGearbox1, true);
    }

    private int VisionSetAngle(double distance) {
        int angle = 0;
        if (distance > 0 && distance < 5) {
            angle = (int) (-166.49 * Math.pow(distance, -0.51321) + 194.95);
        }
        return angle;
    }


    // Beak Subsystem
    public void Intake() {
        if (!bottomLimitSwitch.get() && !intakeLimitSwitch.get() && currentNeckState == NeckStates.INTAKE) {
            bumperIntakeMotor.set(1.0);
            beakIntakeMotor.set(-1.0);
        } else {
            IntakeStop();
        }
    }

    private void IntakeCheck() {
        if (bottomLimitSwitch.get() && subwooferPositionSet && !intakeLimitSwitch.get() && currentNeckState != NeckStates.INTAKE) {
            beakIntakeMotor.set(-1.0);
        } else if (intakeLimitSwitch.get()) {
            IntakeStop();
            downTimerStarted = false;
            downTimer.stop();
            downTimer.reset();
        }
    }
    
    public void Outtake() {
        bumperIntakeMotor.set(-1.0);
        beakIntakeMotor.set(1.0);
        subwooferPositionSet = false;
    }

    public void IntakeStop() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
    }

    public void Shooter() {
        if (currentNeckState == NeckStates.AMP) {
            beakIntakeMotor.set(-1.0);
        } else {
            shooterTopMotor.set(1.0);
            shooterBottomMotor.set(1.0);

            if (!shooterTimerStarted) {
                shooterTimer.start();
                shooterTimerStarted = true;
            } else if (shooterTimer.get() >= 0.8) {
                beakIntakeMotor.set(-1.0);
            }
        }
    }
    
    public void ShooterStop() {
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
        beakIntakeMotor.set(0.0);
        shooterTimer.stop();
        shooterTimer.reset();
        shooterTimerStarted = false;
    }

    public void ShooterAuto() {
        shooterTopMotor.set(1.0);
        shooterBottomMotor.set(1.0);
    }

    public void IntakeAuto() {
        bumperIntakeMotor.set(1.0);
        beakIntakeMotor.set(-1.0);
    }

    public void StopAllAuto() {
        bumperIntakeMotor.set(0.0);
        beakIntakeMotor.set(0.0);
        shooterTopMotor.set(0.0);
        shooterBottomMotor.set(0.0);
    }

    public PhotonCamera returnCamera() {
        return camera;
    }
}