package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

public class DriveTrain extends SubsystemBase {

    // private static final Logger LOG = LoggerFactory.getLogger(DriveTrain.class);

    //private PhotonCamera photonCamera;

    private EncoderOffsets encoderOffsets = new EncoderOffsets();

    private PhotonPoseEstimator photonPoseEstimator;

    private WPI_TalonFX backLeft;
    private WPI_TalonFX backRight;
    private WPI_TalonFX frontLeft;
    private WPI_TalonFX frontRight;
    private DifferentialDrive differentialDrive;
    private boolean controlsReversed = false;

    private boolean wantLow = true;

    private AHRS ahrs = new AHRS(Port.kMXP);
    private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(RobotConstants.kTrackWidthMeters);
    private DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, ahrs.getRotation2d(), 0.0, 0.0, new Pose2d());

    private PhotonCamera camera = new PhotonCamera("Front_Camera");
    
    private SupplyCurrentLimitConfiguration frontSupplyLimit = new SupplyCurrentLimitConfiguration(false,
            DriveConstants.driveLowCurrentLimit, DriveConstants.driveLowCurrentLimit,
            DriveConstants.triggerThresholdTime);
    
    final Field2d m_fieldSim = new Field2d();

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

       // m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), 0, 0);

       AprilTagFieldLayout atfl;
    try {
        //atfl = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/frc2023.fmap");
        photonPoseEstimator =
       new PhotonPoseEstimator(
               AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, new Transform3d(
                       new Translation3d(.09, 0, 1.055),
                       new Rotation3d(
                               0, 0,
                               0)));
    } catch (IOException e) {
        // TODO Auto-generated catch block
        
        e.printStackTrace();
    }
            //atfl.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

       
    }

    private WPI_TalonFX createTalonFX(int deviceID, TalonFXInvertType direction) {
        WPI_TalonFX motor = new WPI_TalonFX(deviceID);
        motor.configFactoryDefault();
        motor.configOpenloopRamp(DriveConstants.voltageRampRate);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit,
                DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        motor.setSelectedSensorPosition(0, 0, 10);
        motor.setInverted(direction);
        return motor;
    }

    public void setLowCurrentLimit() {

        frontLeft.configSupplyCurrentLimit(frontSupplyLimit);
        frontRight.configSupplyCurrentLimit(frontSupplyLimit);
    }

    public void setHighCurrentLimit() {
        frontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit,
                DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        backRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit,
                DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        backLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit,
                DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
        frontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DriveConstants.driveCurrentLimit,
                DriveConstants.driveCurrentLimit, DriveConstants.triggerThresholdTime));
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
        backLeft.feed();
        backRight.feed();
        m_poseEstimator.update(ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
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
        return ((frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition()) / 2D)
                - encoderOffsets.frontLeft;
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
        return ((frontRight.getSelectedSensorPosition() + backRight.getSelectedSensorPosition()) / 2D)
                - encoderOffsets.frontRight;
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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                ahrs.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            System.out.println("Found AprilTag");
            EstimatedRobotPose camPose = result.get();
            m_poseEstimator.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            m_fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
        } else {
            System.out.println("No AprilTag");
            // move it way off the screen to make it disappear
            m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        m_fieldSim.getObject("Actual Pos").setPose(getPose());
        m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }


    public void arcadeDrive(double throttle, double steering) {
        // LOG.trace("Throttle = {}, Steering = {}, ControlsReversed = {}", throttle,
        // steering, controlsReversed);
        differentialDrive.arcadeDrive(throttle, steering, false);
    }

    public void arcadeDriveSquared(double throttle, double steering) {
        arcadeDrive(throttle, steering);
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
        // ahrs.reset();
    }

    public double getHeading() {
        //double angle = ahrs.getYaw();
        // LOG.info("NavX Heading: {}", angle);
        //return angle;
        return 0;
    }

    public void stopDriving() {
        tankDrive(0, 0);
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        //m_odometry.resetPosition(ahrs.getRotation2d(), 0, 0, pose);
    }

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

        public void setOffsets(WPI_TalonFX frontLeft, WPI_TalonFX backLeft, WPI_TalonFX frontRight,
                WPI_TalonFX backRight) { 
            this.frontLeft = frontLeft.getSelectedSensorPosition();
            this.backLeft = backLeft.getSelectedSensorPosition();
            this.frontRight = frontRight.getSelectedSensorPosition();
            this.backRight = backRight.getSelectedSensorPosition();
        }
    };

    public void update(double leftDist, double rightDist) {
        m_poseEstimator.update(ahrs.getRotation2d(),  leftDist, rightDist);

        PhotonPipelineResult res = camera.getLatestResult();
        if (res.hasTargets()) {
            System.out.println("I see a target!");
            int ids = res.targets.size();
            for (int id_num = 0;id_num < ids;id_num ++) {
                PhotonTrackedTarget target = res.targets.get(id_num);
                double imageCaptureTime = res.getTimestampSeconds();
                Transform3d camToTargetTrans = target.getBestCameraToTarget();
                int id = target.getFiducialId();
                Pose3d originalPose;
                switch(id) {
                    case 1:
                        originalPose = Constants.Targets.target1;
                        break;
                    case 2:
                        originalPose = Constants.Targets.target2;
                        break;
                    case 3:
                        originalPose = Constants.Targets.target3;
                        break;
                    case 4:
                        originalPose = Constants.Targets.target4;
                        break;
                    case 5:
                        originalPose = Constants.Targets.target5;
                        break;
                    case 6:
                        originalPose = Constants.Targets.target6;
                        break;
                    case 7:
                        originalPose = Constants.Targets.target7;
                        break;
                    case 8:
                        originalPose = Constants.Targets.target8;
                        break;
                    default:
                        return; // it will NEVER get here, no worries.
                }

                Pose3d camPose = originalPose.transformBy(camToTargetTrans.inverse());
                m_poseEstimator.addVisionMeasurement(camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
            }

        }
            
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

}
