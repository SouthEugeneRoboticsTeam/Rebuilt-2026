package org.sert2521.rebuilt2026.subsystems.drivetrain

import com.ctre.phoenix6.configs.MountPoseConfigs
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import limelight.Limelight
import limelight.networktables.LimelightPoseEstimator
import limelight.networktables.Orientation3d
import org.sert2521.rebuilt2026.commands.JoystickDrive
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import kotlin.math.PI
import kotlin.math.hypot

object Drivetrain : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder: CANcoder,
        moduleName: String, location: Translation2d, rotationZero: Angle
    ): SwerveModule {
        val driveConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withWheelDiameter(SwerveConstants.wheelRadius.times(2.0))
            .withFeedforward(SimpleMotorFeedforward(SwerveConstants.DRIVE_S, SwerveConstants.DRIVE_V))
            .withClosedLoopController(SwerveConstants.DRIVE_P, 0.0, SwerveConstants.DRIVE_D)
            .withGearing(SwerveConstants.driveGearing)
            .withOpenLoopRampRate(Seconds.zero())
            .withClosedLoopRampRate(Seconds.zero())
            .withMotorInverted(true)
            .withStatorCurrentLimit(SwerveConstants.driveCurrentLimit)
            .withTelemetry("$moduleName Drive Motor", SmartMotorControllerConfig.TelemetryVerbosity.MID)

        val angleConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withGearing(SwerveConstants.angleGearing)
            .withClosedLoopController(SwerveConstants.ANGLE_P, 0.0, SwerveConstants.ANGLE_D)
            .withContinuousWrapping(-Radians.of(PI), Radians.of(PI))
            .withStatorCurrentLimit(SwerveConstants.angleCurrentLimit)
            .withOpenLoopRampRate(Seconds.zero())
            .withClosedLoopRampRate(Seconds.zero())
            .withMotorInverted(true)
            .withTelemetry("$moduleName Angle Motor", SmartMotorControllerConfig.TelemetryVerbosity.MID)

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

    private val gyroConfig = MountPoseConfigs().withMountPoseRoll(Degrees.of(-5.93248176574707)).withMountPoseYaw(Degrees.of(118.74869537353516)).withMountPosePitch(Degrees.of(-79.23535919189453))
    private val gyro = Pigeon2(13)
    private val gyroYaw = gyro.yaw.asSupplier()
    private val gyroPitch = gyro.pitch.asSupplier()
    private val gyroRoll = gyro.roll.asSupplier()

    private val kinematics = SwerveDriveKinematics(*SwerveConstants.moduleTranslations)
    private var moduleStates = Array(4) { modules[it].state }

    private val moduleLock = { false }
    private var moduleLockPrevReference = Array(4) { Rotation2d.kZero }

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(gyroYaw.get()),
        Array(4) { modules[it].position },
        Pose2d.kZero
    )

    private val limelight = Limelight("limelight")
    private val limelightPoseEstimator = limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2)

    private val field = Field2d()

    init {
        SmartDashboard.putData(field)

        defaultCommand = JoystickDrive()

        gyro.configurator.apply(gyroConfig)
        poseEstimator.setVisionMeasurementStdDevs(VisionConstants.visionStdv)

        DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", Array(4) { SwerveModuleState() })
        DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", Array(4) { SwerveModuleState() })
    }

    override fun periodic() {
        modules.forEach { it.updateTelemetry() }
        modules.forEach { it.seedAzimuthEncoder() }

        poseEstimator.update(Rotation2d(gyroYaw.get()), getModulePositions())

        moduleStates = getModuleStates()
        DogLog.log("Drivetrain/SwerveModuleStates/Measured", moduleStates)

        val chassisSpeeds = kinematics.toChassisSpeeds(*moduleStates)
        DogLog.log("Drivetrain/ChassisSpeeds/Measured", chassisSpeeds)
        DogLog.log(
            "Drivetrain/ChassisSpeeds/Measured Drive Speed",
            hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        )

        DogLog.log("Drivetrain/Yaw", gyroYaw.get())
        DogLog.log("Drivetrain/Pitch", gyroPitch.get())
        DogLog.log("Drivetrain/Roll", gyroRoll.get())

        updateVision()

        field.robotPose = poseEstimator.estimatedPosition
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

    private fun setModuleStates(vararg states: SwerveModuleState): Array<SwerveModuleState> {
        modules.forEachIndexed { index, module ->
            if (moduleLock()) {
                states[index].angle = moduleLockPrevReference[index]
                module.setSwerveModuleState(states[index])
            } else {
                module.setSwerveModuleState(states[index])
            }
        }
        return Array(4) { modules[it].config.getOptimizedState(states[it]) }
    }

    fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { modules[it].position }
    }

    fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { modules[it].state }
    }

    fun updateVision(){
        limelight.settings.withRobotOrientation(
            Orientation3d(
                Rotation3d(gyroRoll.get(), gyroPitch.get(), gyroYaw.get()),
                RotationsPerSecond.zero(),
                RotationsPerSecond.zero(),
                RotationsPerSecond.zero()
            )
        )

        val estimatedPose = limelightPoseEstimator.poseEstimate
        if (estimatedPose.isEmpty) {
            return
        }
        if (estimatedPose.get().pose.rotation.measureX > VisionConstants.rotationThreshold){
            return
        }
        if (estimatedPose.get().pose.rotation.measureY > VisionConstants.rotationThreshold){
            return
        }

        poseEstimator.addVisionMeasurement(estimatedPose.get().pose.toPose2d(), Timer.getFPGATimestamp())
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

    fun getChassisSpeeds():ChassisSpeeds{
        return kinematics.toChassisSpeeds(*moduleStates)
    }

    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetPose(pose)
    }

    fun setRotation(rotation: Rotation2d) {
        poseEstimator.resetRotation(rotation)
    }

    fun setModulePrevReference(){
        moduleLockPrevReference = Array(4) { moduleStates[it].angle }
    }

    fun runFFCharacterization(output:Double){
        modules.forEach {
            it.driveMotorController.voltage = Volts.of(output)
            it.azimuthMotorController.setPosition(Rotations.of(0.0))
        }
    }

    fun getFFCharacterizationVelocity():Double{
        var output = 0.0
        modules.forEach {
            output += it.driveMotorController.mechanismVelocity.`in`(RotationsPerSecond)
        }
        return output / 4.0
    }
}