package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Utils;
import edu.wpi.first.wpilibj.I2C;

import static frc.robot.Constants.BigIronConstants.*;
import static frc.robot.Constants.ShooterData;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Launcher {
    // motors
    private static final CANSparkMax intakeMotor = new CANSparkMax(kIntakeID, MotorType.kBrushless);
    private static final CANSparkMax drumOneMotor = new CANSparkMax(kDrumOneID, MotorType.kBrushless);
    private static final CANSparkMax drumTwoMotor = new CANSparkMax(kDrumTwoID, MotorType.kBrushless);
    private static final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private static final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoderPin);
    private static final TalonSRX beltMotor = new TalonSRX(kBeltID);

    // solenoids
    private static final DoubleSolenoid intakeForward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, kChannelIntakeForwardGo, kChannelIntakeForwardVent);
    private static final DoubleSolenoid intakeBackward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, kChannelIntakeBackwardGo, kChannelIntakeBackwardVent);
    private static final Value kGo = Value.kForward;
    private static final Value kVent = Value.kReverse;

    // Sensors
    private static final DigitalInput breachSensor = new DigitalInput(kBreachSensorPin);
    private static final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
    private static final AnalogInput pressureSensor = new AnalogInput(3);// airTank sensor
    private static final DigitalInput hoodLimit = new DigitalInput(kHoodDownLimitPin);

    static double tankPressure;

    // Public values
    public static boolean fireTheBigIron = false;
    public static boolean robotAligned = false;
    public static boolean drumControllerOn = false;
    public static boolean complianceOverride = false;
    public static boolean climbing = false;
    public static boolean drumIdle = false;
    public static double drumCurrentSpeed = 0;
    public static double drumSP = 2000;
    public static boolean lowShot = false;
    public static boolean runBeltMan = false;
    public static boolean runHoodMan = false;
    public static String ball1Col = "null";
    public static String ball2Col = "null";
    public static double hoodManPower = 0;
    public static int ballCount = 0;
    public static String colorOurs;
    public static String colorAnti;

    // Misc.
    private static final Timer lowShotTimer = new Timer();

    // Private process variables
    private static boolean calibrated = false;
    public static double hoodSet = 0.05;
    private static double hoodCurrentPosition = 0;
    private static boolean hoodLowLimit = false;
    public static boolean ballOnTheWay = false;
    public static boolean breachSensorFlag = false;
    private static boolean intakeSensorFlag = false;


    public static void init() {
        dPID.reset();
        hPID.reset();
        hoodMotor.configNeutralDeadband(0);
        intakeOut = false;

        // drum motor stuff
        //drumLeadMotor.restoreFactoryDefaults();
        //drumFollowMotor.restoreFactoryDefaults();
        drumOneMotor.setInverted(false);
        drumTwoMotor.setInverted(false);
        drumTwoMotor.follow(drumOneMotor,true);
    }

    /** blanket reset */
    public static void reset() {
        dPID.reset();
        hPID.reset();
        beltTimer.stop();
        beltTimer.reset();
        lowShotTimer.stop();
        lowShotTimer.reset();
        lowShot = false;
        calibrated = false;
        //drumIdle = false;
        ball1Col = "null";
        ball2Col = "null";
        ballOnTheWay = false;
        ballCount = 0;
        intakeSensorFlag = false;
        breachSensorFlag = false;
        fireTheBigIron = false;
        lf = false;
        lff = false;
        beltMotor.set(ControlMode.PercentOutput, 0);
        drumOneMotor.set(0);
        drumTwoMotor.set(0);
    }

        /**
     * if the ball count is zero, it will trigger the intake artificially,
     * can also be used to stop belt
     * if bc == 1, it will wind both balls to the top
     */
    public static void increaseBC() {
        if (ballCount == 0) {
            ballOnTheWay = true;
            woundBack = false;
            beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 1) ballCount++;
    }

    /**
     * 
     */
    public static void decreaseBC() {
        if (ballCount == 1) {
            ballCount = 0;
            ballOnTheWay = false;
            woundBack = false;
            lowShot = false;
            lf = false;
            lff = false;
        } else if (ballCount == 2) {
            ballCount--;
            woundBack = false;
        }
    }

    public static void afterFiring() {
        ballCount = 0;
        drumIdle = false;
        ballOnTheWay = false;
        woundBack = true;
        lowShot = false;
        lf = false;
        lff = false;
        fireTheBigIron = false;
        dPID.reset();
    }

    /** set intake motor
     * @deprecated does nothing
     * @param b intake on/off
     */
    public void runIntake(boolean b) {
        if (intakeOut && b) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    /** Checks for proper alignment, drumspeed, and hood position
     * @return whether or not the robot is completely ready to fire
     */
    public static boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    public static boolean intakeOut = true;
    private static Timer t = new Timer();
    /**
     * Refresh the intake logic, sets intake in/out and controls compliance
     * @param b a toggle for the intake retract and deploy, meant to be used with controller.getRawButtonPressed(k); inside the {@link frc.robot.commands.M_Teleop} command
     */
    public static void intakeUpdate(boolean b) {
        if (b) {
            if (!intakeOut) {
                intakeOut = true;
                t.start();
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeOut = false;
                t.stop();
                t.reset();
                intakeForward.set(kVent);
                intakeBackward.set(kGo);
            }
        } else if (t.hasElapsed(1)) {
            if (complianceOverride) {
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeForward.set(kVent);
                intakeBackward.set(kVent);
            }
        }
        if (intakeOut) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    /** Sets the drum speed and hood position by interpolating between points in the {@link ShooterData}. 
     * The key to TyRapXXII's shooter.
     * @param m the distance to the target in meters as reported by the limelight
     * @author Carl C.
     */
    public static void setAimDistance(double m) {
        int i = ShooterData.distances.length-1;
        for (int j = 1; j < ShooterData.distances.length; j++) {
            if (m < ShooterData.distances[j]) {
                i = j;
                break;
            }
        }
        double upper = ShooterData.distances[i];
        double lower = ShooterData.distances[i-1];
        double lerpFactor = (m-lower)/Math.abs(upper-lower);
        upper = ShooterData.drumSpeeds[i];
        lower = ShooterData.drumSpeeds[i-1];
        drumSP = (int)Utils.lerpA(lower, upper, lerpFactor);
        upper = ShooterData.hoodPositions[i];
        lower = ShooterData.hoodPositions[i-1];
        hoodSet = Utils.lerpA(lower, upper, lerpFactor);
    }

    public static void update() {
        readSensors();
        logic();
        runHood();
        runDrum();
        runFeedBelt();
    }

    /** read the sensors and update their flags */
    private static void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
        breachSensorFlag = !breachSensor.get();
        double inSens = intakeSensor.getProximity();
        intakeSensorFlag = (inSens > 310 || inSens < 230) && intakeOut;
        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        tankPressure = 250 * (pressureSensor.getVoltage() / 5) - 25;
        hoodLowLimit = !hoodLimit.get();
        drumCurrentSpeed = -1*drumOneMotor.getEncoder().getVelocity();
    }

    private static boolean lf = false;
    private static boolean lff = false;
    /** use for general state logic */
    private static void logic() {
        if (lowShot) {
            if (!lf) {
                lowShotTimer.reset();
                lowShotTimer.start();
                lf = true;
                lff = false;
            }
        }
    }

    /** calibrate and control hood position */
    private static void runHood() {
        if (calibrated) {
            double control = hALG.get(hoodCurrentPosition, hoodSet);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else {
                hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, 0, 1));// limit that hood thing
                hPID.reset();
                hoodEncoder.reset();
            }
        } else { // if not calibrated run hood down until limit is triggered
            if (hoodLowLimit) {
                calibrated = true;
                hPID.reset();
                hoodEncoder.reset();
            }
            else {
                hoodMotor.set(ControlMode.PercentOutput, -0.55);
            }
        }
        //hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    /** Update and run drum speed pid loops */
    private static void runDrum() {
        if (fireTheBigIron || drumIdle) drumOneMotor.set(dALG.get(drumCurrentSpeed, drumSP));
        else if (lowShot || lff) drumOneMotor.set(kDrumEjectPower);
        else {
            drumOneMotor.set(0);
            dPID.reset();
        }
    }

    private static Timer beltTimer = new Timer();
    private static boolean woundBack = true;

    /** The most complex piece of code in the robot, is utterly absurd. Took forever to get working and 
     * is borderline black magic. This method controls how the feed belt is run and when. It also contains 
     * the logic that increases/decreases the ball count. Changing any part of this could completely 
     * break the robot's ability to intake balls and shoot them.
     */
    private static void runFeedBelt() {
        if (fireTheBigIron) { // if firing, run belt as long as the robot is ready to fire
            if (readyToFire() && robotAligned) {
                beltMotor.set(ControlMode.PercentOutput, kBeltPower); 
            } else beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 0) {
            if (!ballOnTheWay) { // if the ball count is zero, wait for the intake to trigger to set the ballontheway flag
                if (!woundBack) {
                    beltTimer.start();
                    if (!beltTimer.hasElapsed(0.6)) {
                        beltMotor.set(ControlMode.PercentOutput, -kBeltPower);
                    } else {
                        beltTimer.stop();
                        beltTimer.reset();
                        woundBack = true;
                        beltMotor.set(ControlMode.PercentOutput, 0);
                    }
                } else {
                    if (!breachSensorFlag && intakeSensorFlag) {
                        ballOnTheWay = true;
                        beltTimer.start();
                        if (ball1Col.equals("null")) ball1Col = getColor();
                    }
                    beltMotor.set(ControlMode.PercentOutput, 0);
                }
            } else { // if a ball is on the way, run it up until the breach sensor is triggered
                beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                if (breachSensorFlag || beltTimer.hasElapsed(3)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    ballOnTheWay = false;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    if (breachSensorFlag) {
                        woundBack = false;
                        ballCount++;
                    }
                }
            }
        } else if (ballCount == 1) {
            if (!woundBack) { // if the ball hasn't been wound back, wind it back for 0.6 seconds
                if (breachSensorFlag) beltTimer.start();
                beltMotor.set(ControlMode.PercentOutput, kBeltReversePower);
                if (beltTimer.hasElapsed(0.6)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    woundBack = true;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                }
            } else { // if it is wound back: 
                if (intakeSensorFlag) { //increase ball count if intake is triggered
                    ballCount++;
                } else if (lowShot) { // if it needs to eject, wind until either the breach sensor is triggered, or if 0.5 seconds has passed
                    if (breachSensorFlag) {
                        if (lowShotTimer.hasElapsed(0.5) && lf) {
                            beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                            lff = true;
                            lowShot = false;
                        }
                        else beltMotor.set(ControlMode.PercentOutput, 0);
                    } else beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                } else if (lff) { // this eff flag keeps the belt running once the 0.5 seconds has passed and is ready to eject
                    if (!breachSensorFlag) {
                        beltMotor.set(ControlMode.PercentOutput, 0);
                        reLoad();
                        lf = false;
                        lff = lf;
                    }
                } else beltMotor.set(ControlMode.PercentOutput, 0); 
            }
        } else if (ballCount == 2) {
            if (ball2Col.equals("null")) ball2Col = getColor();
            else if (lowShot) { // if ejecting, run similar code to the eject code for one ball
                if (lowShotTimer.hasElapsed(0.5) && lf) {
                    beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                    lowShot = false;
                    lff = true;
                } else beltMotor.set(ControlMode.PercentOutput, 0);
            } else if (lff) { // is more complex as it needs to feed into the 1 ball eject
                if (!breachSensorFlag) {
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    ball1Col = ball2Col;
                    ball2Col = "null";
                    ballCount = 1;
                    lf = false;
                    lff = lf;
                    fireTheBigIron = false;
                    lowShot = true;
                    woundBack = true;
                }
            } else beltMotor.set(ControlMode.PercentOutput, 0);  
            if (!breachSensorFlag) beltMotor.set(ControlMode.PercentOutput, kBeltPower); // make sure balls are wound up and in firing position
        } 
    }

    /**prepares ball chute for next shot*/
    public static void reLoad() {
        ball1Col = ball2Col;
        if (ballCount > 1) {
            ball2Col = "null";
            ballOnTheWay = true;
        }
        ballCount = 0;
    }
    
    /** get the color of the ball; does not work with enough accuracy; ball color isn't used anywhere
     * @return a string for the color, "Blue" or "Red"
     */
    private static String getColor() {
        double b = intakeSensor.getBlue();
        double r = intakeSensor.getRed();
        if (b > r) return "Blue";
        return "Red";
    }
}
