package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class DriveTrain extends SubsystemBase {

    // private static final Logger LOG = LoggerFactory.getLogger(DriveTrain.class);

    private boolean controlsReversed = false;


    private boolean wantLow = true;


    private class EncoderOffsets {
        public double frontLeft;
        public double backLeft;
        public double frontRight;
        public double backRight;

        EncoderOffsets() {
            frontLeft = 0;
            backLeft = 0;
            frontRight = 0;
            backRight = 0;
        }

        public void setOffsets(WPI_TalonFX frontLeft, WPI_TalonFX backLeft, WPI_TalonFX frontRight, WPI_TalonFX backRight) {
            this.frontLeft = frontLeft.getSelectedSensorPosition();
            this.backLeft = backLeft.getSelectedSensorPosition();
            this.frontRight = frontRight.getSelectedSensorPosition();
            this.backRight = backRight.getSelectedSensorPosition();
        }
    };

    private EncoderOffsets encoderOffsets = new EncoderOffsets();

    private AHRS ahrs = new AHRS(Port.kMXP);

    private WPI_TalonFX backLeft;
    private WPI_TalonFX backRight;
    private WPI_TalonFX frontLeft;
    private WPI_TalonFX frontRight;
    private DifferentialDrive differentialDrive;
    private DifferentialDriveOdometry m_odometry;

    private SupplyCurrentLimitConfiguration frontSupplyLimit = new SupplyCurrentLimitConfiguration(false, DriveConstants.driveLowCurrentLimit, DriveConstants.driveLowCurrentLimit, DriveConstants.triggerThresholdTime);

    public DriveTrain() {
        frontLeft = createTalonFX(DriveConstants.kLeftMotorFrontPort, TalonFXInvertType.Clockwise);
        backLeft = createTalonFX(DriveConstants.kLeftMotorRearPort, TalonFXInvertType.Clockwise);
        frontRight = createTalonFX(DriveConstants.kRightMotorFrontPort, TalonFXInvertType.CounterClockwise);
        backRight = createTalonFX(DriveConstants.kRightMotorRearPort, TalonFXInvertType.CounterClockwise);

        backRight.follow(frontRight);
        backLeft.follow(frontLeft);

        setHighCurrentLimit();
        setLowCurrentLimit();

        differentialDrive = new DifferentialDrive(frontLeft, frontRight);
        addChild("DifferentialDrive", differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(),0,0);
    }
    
    private WPI_TalonFX createTalonFX(int deviceID, TalonFXInvertType direction) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceID);
        motor.configFactoryDefault();
        motor.configOpenloopRamp(DriveConstants.voltageRampRate);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit, DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        motor.setSelectedSensorPosition(0, 0, 10);
        motor.setInverted(direction);



        return motor;
    }

    public void setLowCurrentLimit(){

        frontLeft.configSupplyCurrentLimit(frontSupplyLimit);
        frontRight.configSupplyCurrentLimit(frontSupplyLimit);
       }

    public void setHighCurrentLimit() {
        frontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit, DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        backRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit, DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        backLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit, DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        frontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit, DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
    }

    public boolean isLowLimitEnabled() {
        return frontSupplyLimit.enable;
    }

    public void LowLimitEnable(boolean enable) {
        frontSupplyLimit.enable = enable;
        setLowCurrentLimit();
    }

    public void requestCurrentLimit(boolean wantLow) {
    this.wantLow = wantLow;

    }

    public boolean isLowCurrentRequested() {
        return wantLow;

    }


    public void setBrakeMode() {
        frontLeft.setNeutralMode(NeutralMode.Brake);
        frontRight.setNeutralMode(NeutralMode.Brake);
        backLeft.setNeutralMode(NeutralMode.Brake);
        backRight.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastMode() {
        frontLeft.setNeutralMode(NeutralMode.Coast);
        frontRight.setNeutralMode(NeutralMode.Coast);
        backLeft.setNeutralMode(NeutralMode.Coast);
        backRight.setNeutralMode(NeutralMode.Coast);
    }

    public boolean isControlsReversed() {
        return controlsReversed;
    }

    @Override
    public void periodic() {
        differentialDrive.feed();
        m_odometry.update(ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftMetersPerSecond(), getRightMetersPerSecond());
    }

    public double getLeftEncoderPosition() {
        return ((frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition()) / 2D) - encoderOffsets.frontLeft;
    }

    public double getLeftEncoderDistance() {
        return getLeftEncoderPosition() * RobotConstants.kDistancePerTick;
    }

    public double getLeftEncoderVelocity() {
        return frontLeft.getSelectedSensorVelocity();
    }

    public double getLeftMetersPerSecond() {
        SmartDashboard.putNumber("left speed", getLeftEncoderVelocity() * RobotConstants.kDistancePerTick * 10);
        return getLeftEncoderVelocity() * RobotConstants.kDistancePerTick * 10;
    }

    public double getRightEncoderPosition() {
        return ((frontRight.getSelectedSensorPosition() + backRight.getSelectedSensorPosition()) / 2D) - encoderOffsets.frontRight;
    }

    public double getRightEncoderDistance() {
        return getRightEncoderPosition() * RobotConstants.kDistancePerTick;
    }

    public double getRightEncoderVelocity() {
        return frontRight.getSelectedSensorVelocity();
    }

    public double getRightMetersPerSecond() {
        SmartDashboard.putNumber("right speed", getRightEncoderVelocity() * RobotConstants.kDistancePerTick * 10);
        return getRightEncoderVelocity() * RobotConstants.kDistancePerTick * 10;
    }

    public double getAverageEncoderPosition() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
    }

    public void resetEncoders() {
        encoderOffsets.setOffsets(frontLeft, backLeft, frontRight, backRight);
    }

    public void arcadeDrive(double throttle, double steering) {
        // LOG.trace("Throttle = {}, Steering = {}, ControlsReversed = {}", throttle, steering, controlsReversed);
        differentialDrive.arcadeDrive(throttle, steering);
    }

    public void curvatureDrive(double throttle, double rotation, boolean turnInPlace) {
        DifferentialDrive.curvatureDriveIK(throttle, rotation, turnInPlace);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        frontLeft.setVoltage(leftVolts);    
        frontRight.setVoltage(rightVolts);
    }

    public void resetHeading() {
        // LOG.info("Reseting Heading...");
        ahrs.reset();
    }

    public double getHeading() {
        double angle = ahrs.getYaw();
        // LOG.info("NavX Heading: {}", angle);
        return angle;
    }

    public void stopDriving() {
        tankDrive(0, 0);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(ahrs.getRotation2d(),0, 0, pose);
    }
}