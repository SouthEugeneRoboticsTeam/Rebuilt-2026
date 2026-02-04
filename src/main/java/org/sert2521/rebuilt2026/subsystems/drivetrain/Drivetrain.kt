package org.sert2521.rebuilt2026.subsystems.drivetrain

import com.ctre.phoenix6.configs.MountPoseConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import kotlin.math.PI
import kotlin.math.hypot

object Drivetrain : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder:CANcoder,
        moduleName:String, location: Translation2d, rotationZero: Angle
    ):SwerveModule{
        val driveConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withWheelDiameter(SwerveConstants.wheelRadius.times(2.0))
            .withFeedforward(SimpleMotorFeedforward(SwerveConstants.DRIVE_S, SwerveConstants.DRIVE_V))
            .withClosedLoopController(SwerveConstants.DRIVE_P, 0.0 ,SwerveConstants.DRIVE_D)
            .withGearing(SwerveConstants.driveGearing)
            .withMotorInverted(true)
            .withStatorCurrentLimit(SwerveConstants.driveCurrentLimit)
            .withTelemetry("$moduleName Drive Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val angleConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withGearing(SwerveConstants.angleGearing)
            .withClosedLoopController(SwerveConstants.ANGLE_P, 0.0, SwerveConstants.ANGLE_D)
            .withContinuousWrapping(-Radians.of(PI), Radians.of(PI))
            .withStatorCurrentLimit(SwerveConstants.angleCurrentLimit)
            .withMotorInverted(true)
            .withTelemetry("$moduleName Angle Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

        val driveSMC = SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig)
        val angleSMC = SparkWrapper(angleMotor, DCMotor.getNEO(1), angleConfig)

        val moduleConfig = SwerveModuleConfig(driveSMC, angleSMC)
            .withAbsoluteEncoderOffset(rotationZero)
            .withAbsoluteEncoder(
                absoluteEncoder.position.asSupplier()
            )
            .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.LOW)
            .withLocation(location)
            .withOptimization(true)
            .withCosineCompensation(false)

        return SwerveModule(moduleConfig)
    }

    private val modules = Array(4) { index ->
        createModule(
            SparkMax(SwerveConstants.driveIDs[index], SparkLowLevel.MotorType.kBrushless),
            SparkMax(SwerveConstants.angleIDs[index], SparkLowLevel.MotorType.kBrushless),
            CANcoder(SwerveConstants.encoderIDs[index]),
            SwerveConstants.moduleNames[index],
            SwerveConstants.moduleTranslations[index],
            SwerveConstants.moduleZeroRotations[index]
        )
    }

    private val gyroConfig = MountPoseConfigs().withMountPoseRoll(Degrees.of(-90.0))
    private val gyro = Pigeon2(13)
    private val gyroYaw = gyro.yaw.asSupplier()

    private val kinematics = SwerveDriveKinematics(*SwerveConstants.moduleTranslations)
    private var moduleStates = Array(4) { modules[it].state }

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.kZero,
        Array(4) { modules[it].position },
        Pose2d.kZero
    )

    // TODO: Add limelight hopefully

    private val field = Field2d()

    init {
        SmartDashboard.putData(field)

        gyro.configurator.apply(gyroConfig)
    }

    override fun periodic() {
        modules.forEach { it.updateTelemetry() }
        modules.forEach { it.seedAzimuthEncoder() }

        poseEstimator.update(Rotation2d(getGyroAngle()), getModulePositions())

        moduleStates = getModuleStates()
        DogLog.log("Drivetrain/SwerveModuleStates/Measured", moduleStates)

        val chassisSpeeds = kinematics.toChassisSpeeds(*moduleStates)
        DogLog.log("Drivetrain/ChassisSpeeds/Measured", chassisSpeeds)
        DogLog.log(
            "Drivetrain/ChassisSpeeds/Measured Drive Speed",
            hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        )

        DogLog.log("Drivetrain/Rotation", poseEstimator.estimatedPosition.rotation)

        if (DriverStation.isDisabled()) {
            DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", Array(4) { SwerveModuleState() })
            DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", Array(4) { SwerveModuleState() })
        }
    }

    fun driveRobotRelative(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        DogLog.log("Drivetrain/ChassisSpeeds/Setpoints", speeds)
        DogLog.log(
            "Drivetrain/ChassisSpeeds/Setpoint Drive Speed",
            hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        )

        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.maxSpeed)

        DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", setpointStates)

        // Modules are set here
        val optimizedStates = setModuleStates(*setpointStates)
        DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", optimizedStates)
    }

    fun getGyroAngle():Angle{
        return gyroYaw.get()
    }

    private fun setModuleStates(vararg states: SwerveModuleState): Array<SwerveModuleState> {
        modules.forEachIndexed { index, module ->
            module.setSwerveModuleState(states[index])
        }
        return Array(4) { modules[it].config.getOptimizedState(states[it]) }
    }

    fun getModulePositions():Array<SwerveModulePosition>{
        return Array(4){ modules[it].position }
    }

    fun getModuleStates():Array<SwerveModuleState>{
        return Array(4){ modules[it].state }
    }

    fun stopDrivePID() {
        modules.forEach {
            it.config.driveMotor.setFeedback(0.0, 0.0, 0.0)
        }
    }

    fun startDrivePID() {
        modules.forEach {
            it.config.driveMotor.setFeedback(SwerveConstants.DRIVE_P, 0.0, SwerveConstants.DRIVE_D)
        }
    }

    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    fun setRotation(rotation: Rotation2d){
        poseEstimator.resetRotation(rotation)
    }
}