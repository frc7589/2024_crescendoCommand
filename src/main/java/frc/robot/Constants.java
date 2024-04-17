package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem.DriveMode;

import java.util.HashMap;

/** 常數設定值 */
public class Constants {
    public static final double[] kSendingSetpoints = {0.05, 1.8};

    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(-0.4, 0.5, 0.0), new Rotation3d(0,0,0)); 

    public static class WristConstants {
        public static final int kLeaderMotorID = 49;
        public static final int kFollowerMotorID = 53;

        public static final int kEncoderID = 2;
        public static final int kSwitchPortID = 4;

        public static final double kEncoderOffset = 0.716;

        public static final double kP = 2.1;//3.04318181817//0.68181818181; //0.7090909091; //0.68181818181; //2.82;
        public static final double kI = 0.03111368832; //0.04990909091; //0.010594584;//0.0108108;//0.005809090909;
        public static final double kD = 0; //0.225;
    }

    public static class ElevatorConstants {
        public static final int kLeftMotorID = 43;
        public static final int kRightMotorID = 46;

        public static final int kEncoderID = 1;
        public static final int kSwitchPortID = 3;

        public static final double kEncoderOffset = 0.0;

        public static final double kP = 4.2727272727;//5.09090909091;//11.2;//0.42;//0.94185;//0.68181818181; //0.7090909091; //0.68181818181; //2.82;
        public static final double kI = 0.05383636364; //0.05549090909;//0;//0.0005; //0.010594584;//0.0108108;//0.005809090909;
        public static final double kD = 0;//0; //0.225;

        // Ku = 1.5
        // Tu = 64.0204865625 * 0.028116
    }

    /**
     * Arm 配置常數
     */
    public static class ArmConstants {
        public static final int kLeftMotorID = 48;
        public static final int kRightMotorID = 53;
        public static final int kEncoderID = 0;

        public static double kEncoderOffset = 0.044;//0.115;

        public static double kMaxAngle = 0.4;
        public static double kMinAngle = 0;

        public static final double kP = 3.9545454545; //2.82;
        public static final double kI = 0.07284804219;
        public static final double kD = 0.00000000000001;

        public static final double kInitialAngle = 62;

        public static final double kSpeakerShootingMaxHeight = 1.9; // unit: meter

        public static final double kArmLength = 0.55; // unit: meter
        public static final double kInitialHeight = 0.38; // unit: meter

        public static final HashMap<Integer, Double> angleMap = new HashMap<>();
    }

    /**
     * Conveyor 配置常數
     */
    public static class ConveyorConstants {
        public static final int kIntakeMotorID = 48;
        public static final int kShooterLeftMotorID = 50;
        public static final int kShooterRightMotorID = 44;

        public static final int kSensorPortID = 0;

        public static final double kShooterOutput = 0.62;
        public static final double kIntakeOutput = 0.4;

        public static final double kShooterSpeed = 3400;
        public static final double kShooterRadius = 5.08; // unit: centimeter

        public static final double kMinIntakePausepoint = 76;
    }

    /**
     * Swerve 配置常數
     */
    public static class SwerveDriveConstants {
        // Default Driving Mode
        public static final DriveMode kDefaultDriveMode = DriveMode.Field;

        public static final double kDefaultSpeed = 0.5;

        public static final int kDefaultDriveSpeedIndex = 1;
        public static final double[] kDriveSpeeds = {0.3, 0.5, 0.7, 0.9, 1.0};

        // FL(前左)模組常數
        public static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig(
            57,
            33,
            1,
            -42.6269+180
        );
        //----
        
        // FR(前右)模組常數
        public static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig(
            61,
            62,
            2,
            140.186-0.8789+180
        );
        //----

        // RL(後左)模組常數
        public static final SwerveModuleConfig kRearLeft = new SwerveModuleConfig(
            59,
            56,
            4,
            -82.6171+1.845+180
        );

        // RR(後右)模組常數
        public static final SwerveModuleConfig kRearRight = new SwerveModuleConfig(
            34,
            55,
            3,
            -39.551+0.04+180
        );

        /**
         * 模組間距(unit: meter)
         */
        public static final double kFrameWidth = 0.60325;
        public static final double kFrameLength = 0.60325;

        /**
         * 底牌Swerve運動學計算配置
         */
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(kFrameWidth/2, kFrameLength/2),
            new Translation2d(kFrameWidth/2, -kFrameLength/2),
            new Translation2d(-kFrameWidth/2, kFrameLength/2),
            new Translation2d(-kFrameWidth/2, -kFrameLength/2)
        );

        /**
         * 旋轉馬達控制器PID參數配置
         */
        public static final double kRotor_kP = 0.008181818182;//0.0095; //0.0085277;//0.008338;
        public static final double kRotor_kI = 0.0001374545455;//0.00048; //0.000552; //0.0004545;
        public static final double kRotor_kD = 0; //0.000011;

        /**
         * 移動PID參數配置
         */
        public static final double kPath_kP = 8;//0.0095; //0.0085277;//0.008338;
        public static final double kPath_kI = 0;//0.00048; //0.000552; //0.0004545;
        public static final double kPath_kD = 0; //0.000011;

        public static final double kPathZ_kP = 1;//0.0095; //0.0085277;//0.008338;
        public static final double kPathZ_kI = 0;//0.00048; //0.000552; //0.0004545;
        public static final double kPathZ_kD = 0; //0.000011;

        /**
         * 最大速度與最大加速度
         */
        public static final double kMaxVelocityMetersPerSecond = 5.0;
        public static final double kMaxAccelerationMetersPerSecond = 3.0;


        public static final HolonomicPathFollowerConfig kPPConfig = 
            new HolonomicPathFollowerConfig(
                new PIDConstants(kPath_kP, kPath_kI, kPath_kD),
                new PIDConstants(kPathZ_kP, kPathZ_kI, kPathZ_kD),
                kMaxVelocityMetersPerSecond,
                Math.sqrt(Math.pow(kFrameWidth, 2)+Math.pow(kFrameLength, 2))/2,
                new ReplanningConfig()
            );

        /**
         * 輪徑
         * 1 inch = 2.54 cm
         * 1 centimeter = 0.01 meter
         */
        public static final double kWheelDiameterMeters = 4*2.54*0.01;

        /**
         * 動力馬達與輪子齒輪比值（動力馬達圈數/輪子齒輪圈數)
         */
        public static final double kThrottleGearRatio = 6.12;

        /**
         * 動力馬達Encoder讀數轉換機器位移數
         */
        public static final double kThrottlePositionConversionFactor = 
            (1/kThrottleGearRatio)*kWheelDiameterMeters*Math.PI;

        /**
         * 動力馬達Encoder讀數轉換機器速度係數
         */
        public static final double kThrottleVelocityConversionFactor = 
            kThrottlePositionConversionFactor/60;

        /**
         * 馬達與Encoder反轉設置
         */
        public static final boolean kThrottleMotorInversion = false;
        public static final SensorDirectionValue kRotorEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;
        public static final boolean kRotorMotorInversion = true;
    }

    /**
     * Swerve 模組配置 Class(類別) 
     */
    public static class SwerveModuleConfig {
        private int throttleId;
        private int rotorId;
        private int rotorEncoderId;
        private double rotorAngleOffset;

        /**P
         * Swerve 模組配置 Class(類別) Constructor(建構子)
         * @param throttleId 動力馬達控制器編號
         * @param rotorId 轉向馬達控制器編號
         * @param rotorEncoderId 轉向軸之旋轉編碼器CAN編號
         * @param rotorAngleOffset 轉向軸之旋轉編碼器絕對模式下角度偏差值(unit: degree)
         */
        public SwerveModuleConfig(
            int throttleId,
            int rotorId,
            int rotorEncoderId,
            double rotorAngleOffset
        ) {
            this.throttleId = throttleId;
            this.rotorId = rotorId;
            this.rotorEncoderId = rotorEncoderId;
            this.rotorAngleOffset = rotorAngleOffset;
        }

        public final int getThrottleID() {
            return throttleId;
        }

        public final int getRotorID() {
            return rotorId;
        }

        public final int getRotorEncoderId() {
            return rotorEncoderId;
        }

        public final double getRotorAngleOffset() {
            return rotorAngleOffset / 360;
        }
    }

    // Spark Max穩壓輸出(unit: Vlot)
    public static final double kVoltageCompensation = 12.0;

    
    /**
     * XboxController 配置常數
     */
    public static class XboxControllerConstants {
        // 底盤Xbox控制器編號
        public static final int kDriveControllerID = 0;

        // 功能Xbox控制器編號
        public static final int kUtilControllerID = 1;

        // Xbox控制器數值忽略基準值
        public static final double kControllerMinValue = 0.07;
    }
}