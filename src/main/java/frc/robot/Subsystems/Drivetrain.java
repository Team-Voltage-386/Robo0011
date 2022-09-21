package frc.robot.Subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** a static drivetrain class that fixes issues caused by using dynamic typing for this sort of thing */
public class Drivetrain {
    private static CANSparkMax frontLeftMotor = new CANSparkMax(kFrontLeft, MotorType.kBrushless);
    private static CANSparkMax frontRightMotor = new CANSparkMax(kFrontRight, MotorType.kBrushless);
    private static CANSparkMax rearLeftMotor = new CANSparkMax(kRearLeft, MotorType.kBrushless);
    private static CANSparkMax rearRightMotor = new CANSparkMax(kRearRight, MotorType.kBrushless);

    private static RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
    private static RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

    private static DifferentialDrive diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    private static DoubleSolenoid shifter = new DoubleSolenoid(2, solenoidType, kShiftUp, kShiftDown);

    private static PigeonIMU _pigeon;
    private static PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    // odometry
    private static double[] ypr = new double[3];
    private static Pose2d pos = new Pose2d();
    private static DifferentialDriveOdometry odometry;

    public static void init() {
        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(false);
        rearLeftMotor.follow(frontLeftMotor);// front left yields faulty encoder values so that set follower
        rearRightMotor.follow(frontRightMotor);
        leftEncoder.setPositionConversionFactor(kMPR);
        rightEncoder.setPositionConversionFactor(kMPR);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getRawHeading()));
        _pigeon = new PigeonIMU(kGyro);
    }

    private static double lastYaw = 0;
        public static void update() {
                updateIMU();
                updateOdometry();
                updateWidgets();
                lastYaw = ypr[0];
        }

        /** updates the widgets on the "Main" tab */
        private static void updateWidgets() {
                mainHeading.setDouble(getPose().getRotation().getDegrees());
                mainX.setDouble(getPose().getX());
                mainY.setDouble(getPose().getY());
        }
        private static final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
        private static final NetworkTableEntry mainHeading = mainTab.add("heading",0).withPosition(3, 3).withSize(1, 1).getEntry();
        private static final NetworkTableEntry mainX = mainTab.add("x",0).withPosition(3, 1).withSize(1,1).getEntry();
        private static final NetworkTableEntry mainY = mainTab.add("y",0).withPosition(3, 2).withSize(1,1).getEntry();

        /** resets the encoder positions */
        public static void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }

        /** Passes through to the drivetrain to power the motors
         * @param forwardPower the drive forwards and backwards
         * @param turnPower the left/right rotation
         */
        public static void arcadeDrive(Double forwardPower, Double turnPower) {
                diffDrive.arcadeDrive(forwardPower, turnPower);
        }

        /** a tank drive method */
        public static void tankDrive(Double leftPower, Double rightPower) {
                diffDrive.tankDrive(leftPower, rightPower);
        }

        public static boolean highGear = false;
        /** shifts the transmission into the specified gear
         * @param t true = high, false = low
         */
        public static void setHighGear(Boolean t) {
                if (!t) {
                        shifter.set(DoubleSolenoid.Value.kReverse);
                        leftEncoder.setPositionConversionFactor(kMPR);
                        rightEncoder.setPositionConversionFactor(kMPR);
                        highGear = false;
                }
                else {
                        shifter.set(DoubleSolenoid.Value.kForward);
                        leftEncoder.setPositionConversionFactor(kMPRH);
                        rightEncoder.setPositionConversionFactor(kMPRH);
                        highGear = true;
                }
        }

        /** get the position from the odometery position
         * @return the position in encoder units
         */
        public static Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        /** resets the odometry system, called in {@link frc.robot.Robot} during autonomousInit() */
        public static void resetOdometry(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(pose, Rotation2d.fromDegrees(getRawHeading()));
        }

        /** get the heading from the pidgeon IMU
         * @return the raw heading from the imu in degrees
         */
        public static double getRawHeading() {
                double y = -ypr[0];
                while (y < 0)
                        y += 360;
                while (y > 360)
                        y -= 360;
                return y;
        }

        /** calculates the difference between the input and the actual robot heading
         * @param sp the target heading (degrees)
         * @return the difference between sp and the current imu heading (degrees)
         */
        public static double getHeadingError(double sp) {
                double v = sp - getPose().getRotation().getDegrees();
                while (v < -180)
                        v += 360;
                while (v > 180)
                        v -= 360;
                return v;
        }

        /** updates the local imu rotation values */
        private static void updateIMU() {
                _pigeon.getGeneralStatus(genStatus);
                _pigeon.getYawPitchRoll(ypr);
        }

        /** updates the position of the robot according to the IMU heading and the encoder positions */
        private static void updateOdometry() {
                odometry.update(Rotation2d.fromDegrees(getRawHeading()), leftEncoder.getPosition(),
                                rightEncoder.getPosition());
        }

        /** returns roughly the speed at which the robot is rotating in degrees per second (kind of, not really a unit, just a value) */
        public static double getRotationSpeed() {
                return Math.abs(lastYaw - ypr[0])*50;
        }
}
