package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataContainer;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;

/** Swerve底盤 Class */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final AHRS m_ahrs = new AHRS(Port.kMXP);

    private final SwerveModule m_frontLeft = new SwerveModule(SwerveDriveConstants.kFrontLeft);
    private final SwerveModule m_frontRight = new SwerveModule(SwerveDriveConstants.kFrontRight);
    private final SwerveModule m_rearLeft = new SwerveModule(SwerveDriveConstants.kRearLeft);
    private final SwerveModule m_rearRight = new SwerveModule(SwerveDriveConstants.kRearRight);

    private SwerveDriveOdometry mOdometry;
    private DriveMode mode = SwerveDriveConstants.kDefaultDriveMode;
    private double maxOutput = SwerveDriveConstants.kDefaultSpeed;
    private static SwerveDrivePoseEstimator poseEstimator;

    private double headingOffset = 0;
    private static PIDController pid_zHeading;

    /** Swerve底盤 駕駛模式 */
    public static enum DriveMode {
        Robot("Robot-Oriented"), 
        Field("Field-Oriented");

        private String name;

        private DriveMode(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }

    public static enum ControlMode {
        Manual("Manual"), 
        Field("Field");

        private String name;

        private ControlMode(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }

    public SwerveDriveSubsystem() {
        m_ahrs.reset();
        mOdometry = new SwerveDriveOdometry(
            SwerveDriveConstants.kSwerveKinematics, 
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            getModulePositions()
        );

        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveDriveConstants.kSwerveKinematics,
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            getModulePositions(),
            new Pose2d(1.36, 5.552, new Rotation2d()),
            VecBuilder.fill(1.0, 1.0, 1.0),
            VecBuilder.fill(0.0, 0.0, 0.0)
        );

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::driveChassis,
            SwerveDriveConstants.kPPConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }

              return false;
            },
            this
        );

        pid_zHeading = new PIDController(headingOffset, maxOutput, headingOffset);
    }

    /**
     * 設定駕駛模式
     * @param mode 設定駕駛模式（場地正向、機器正向）
     */
    public Command setMode(DriveMode mode) {
        return runOnce(() -> {
            this.mode = mode;
        });
    }

    /**
     * 依照已設定駕駛模式駕駛
     * @param xSpeed 前後速度
     * @param ySpeed 左右速度
     * @param zSpeed 旋轉速度
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        switch(this.mode) {
            case Field:
                driveField(xSpeed/Math.pow(0.6, Math.abs(xSpeed)-1), ySpeed/Math.pow(0.6, Math.abs(ySpeed)-1), zSpeed/Math.pow(0.3, Math.abs(zSpeed)-1));
                break;

            case Robot:
                driveChassis(xSpeed/Math.pow(0.6, Math.abs(xSpeed)-1), ySpeed/Math.pow(0.6, Math.abs(ySpeed)-1), zSpeed/Math.pow(0.3, Math.abs(zSpeed)-1));
                break;
        }
    }

    /**
     * 依照指定駕駛模式駕駛
     * @param xSpeed 前後速度
     * @param ySpeed 左右速度
     * @param zSpeed 旋轉速度
     * @param mode   駕駛模式
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, DriveMode mode) {
        switch(mode) {
            case Field:
                driveField(xSpeed/Math.pow(0.6, Math.abs(xSpeed)-1), ySpeed/Math.pow(0.6, Math.abs(ySpeed)-1), zSpeed/Math.pow(0.3, Math.abs(zSpeed)-1));
                break;

            case Robot:
                driveChassis(xSpeed/Math.pow(0.6, Math.abs(xSpeed)-1), ySpeed/Math.pow(0.6, Math.abs(ySpeed)-1), zSpeed/Math.pow(0.3, Math.abs(zSpeed)-1));
                break;
        }
    }

    /**
     * 場地正向模式駕駛
     * @param xSpeed 前後速度
     * @param ySpeed 左右速度
     * @param zSpeed 旋轉速度
     */
    public void driveField(double xSpeed, double ySpeed, double zSpeed) {
        SwerveModuleState[] states = SwerveDriveConstants.kSwerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                -zSpeed,
                Rotation2d.fromDegrees(m_ahrs.getAngle()-headingOffset)
            )
        );

        setState(states);
    }

    /**
     * 機器正向模式駕駛
     * @param xSpeed 前後速度
     * @param ySpeed 左右速度
     * @param zSpeed 旋轉速度
     */
    public void driveChassis(double xSpeed, double ySpeed, double zSpeed) {
        SwerveModuleState[] states = SwerveDriveConstants.kSwerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                xSpeed,
                ySpeed,
                -zSpeed
            )
        );

        setState(states);
    }

    public void driveChassis(ChassisSpeeds speeds) {
        driveChassis(
            -speeds.vxMetersPerSecond,
            -speeds.vyMetersPerSecond,
            -speeds.omegaRadiansPerSecond
        );
    }
    

    public ChassisSpeeds getSpeeds() {
        return SwerveDriveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * 取得各模組狀態
     * @return 模組狀態的陣列
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState(),
        };
    }

    /**
     * 取得各模組狀態
     * @return 模組狀態的陣列
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
        };
    }

    /**
     * 設定底盤新的狀態
     * @param newState 期望之模組狀態的陣列
     */
    public void setState(SwerveModuleState[] newState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(newState, maxOutput);

        m_frontLeft.setState(newState[0]);
        m_frontRight.setState(newState[1]);
        m_rearLeft.setState(newState[2]);
        m_rearRight.setState(newState[3]);
    }

    /**
     * 取得機器在場地上的位置
     * @return 機器位置
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * 設定機器在場地上的位置
     * @param pose 位置
     */
    public void setPose(Pose2d pose) {
        mOdometry.resetPosition(
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            getModulePositions(),
            pose
        );

        poseEstimator.resetPosition(Rotation2d.fromDegrees(m_ahrs.getAngle()), getModulePositions(), pose);
    }

    /**
     * 更新位置測量資料
     * 請持續執行
     */
    @Override
    public void periodic() {
        poseEstimator.update(Rotation2d.fromDegrees(m_ahrs.getAngle()), getModulePositions());
        mOdometry.update(
            Rotation2d.fromDegrees(m_ahrs.getAngle()),
            getModulePositions()
        );

        this.updateSmartDashboard();
    }

    /**
     * 重新設定zero angle
     */
    public void resetFieldPositive() {
        m_ahrs.reset();
    }

    /**
     * Reset
     */
    public Command resetHeadingCommand() {
        return runOnce(() -> {
            this.headingOffset = m_ahrs.getAngle();
        });
    }

    /**
     * 更新SmartDashboard上各模組資訊
     */
    public void updateSmartDashboard() {
        SmartDashboard.putString("[Chassis] Driving Mode", this.mode.getName());
        SmartDashboard.putNumber("[Chassis] Max Output", maxOutput);
        SmartDashboard.putNumberArray("[Chassis] Modules' Angle", new double[] {
            this.getModuleStates()[0].angle.getDegrees(),
            this.getModuleStates()[1].angle.getDegrees(),
            this.getModuleStates()[2].angle.getDegrees(),
            this.getModuleStates()[3].angle.getDegrees()
        });
        
        SmartDashboard.putNumberArray("[Chassis] Modules' Speed", new double[] {
            (this.getModuleStates()[0].speedMetersPerSecond)*SwerveDriveConstants.kMaxVelocityMetersPerSecond,
            (this.getModuleStates()[1].speedMetersPerSecond)*SwerveDriveConstants.kMaxVelocityMetersPerSecond,
            (this.getModuleStates()[2].speedMetersPerSecond)*SwerveDriveConstants.kMaxVelocityMetersPerSecond,
            (this.getModuleStates()[3].speedMetersPerSecond)*SwerveDriveConstants.kMaxVelocityMetersPerSecond
        });
        
        SmartDashboard.putNumberArray("[Chassis] Posistion", new double[] {
            poseEstimator.getEstimatedPosition().getX(),
            poseEstimator.getEstimatedPosition().getY(),
            poseEstimator.getEstimatedPosition().getRotation().getDegrees()
        });

        SmartDashboard.putNumberArray("[IMU] Velocitys", new double[] {
            m_ahrs.getVelocityX(),
            m_ahrs.getVelocityY(),
            m_ahrs.getRate()
        });

        SmartDashboard.putNumberArray("[IMU] Accels", new double[] {
            m_ahrs.getWorldLinearAccelX(),
            m_ahrs.getWorldLinearAccelY()
        });
    }

    public Command setMaxOutputCommand(double value) {
        return runOnce(() -> {
            this.maxOutput = value;
        });
    }

    public void setMaxOutput(double value) {
        this.maxOutput = value;
    }

    public Command increaseMaxOutput() {
        return runOnce(() -> {
            if(this.maxOutput > 0.9) return;
            this.maxOutput += 0.1;
        });
    }

    public Command decreaseMaxOutput() {
        return runOnce(() -> {
            if(this.maxOutput < 0.2) return;
            this.maxOutput -= 0.1;
        });
    }

    /**
     * 更新轉向馬達PID控制器常數
     * @param kp 比例單元常數
     * @param ki 積分單元常數
     * @param kd 微分單元常數
     */
    public void updateRotorPID(double kp, double ki, double kd) {
        m_frontLeft.updateRotorPID(kp, ki, kd);
        m_frontRight.updateRotorPID(kp, ki, kd);
        m_rearLeft.updateRotorPID(kp, ki, kd);
        m_rearRight.updateRotorPID(kp, ki, kd);
    }
    
    /**
     * 測試各馬達輸出
     * @param throttle 動力馬達輸出
     * @param rotor    轉向馬達輸出
     */
    public void testRunning(double throttle, double rotor) {
        m_frontLeft.testRunning(throttle, rotor);
        m_frontRight.testRunning(throttle, rotor);
        m_rearLeft.testRunning(throttle, rotor);
        m_rearRight.testRunning(throttle, rotor);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public static double getDistanceToSpeaker() {
        return poseEstimator.getEstimatedPosition().getTranslation().getDistance(new Translation2d(0, 5.552));
    }

    public static Rotation2d getAngleToSpeaker() {
        return poseEstimator.getEstimatedPosition().getTranslation().minus(new Translation2d(0, 5.552)).getAngle();
    }
}
