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
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import limelight.Limelight
import limelight.networktables.LimelightPoseEstimator
import limelight.networktables.Orientation3d
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.TelemetryConstants
import org.sert2521.rebuilt2026.commands.JoystickDrive
import yams.mechanisms.config.SwerveModuleConfig
import yams.mechanisms.swerve.SwerveModule
import yams.motorcontrollers.SmartMotorController
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

object Drivetrain : SubsystemBase() {
    private fun createModule(
        driveMotor: SparkMax, angleMotor: SparkMax, absoluteEncoder: CANcoder,
        moduleName: String, location: Translation2d, rotationZero: Angle
    ): SwerveModule {
        val driveConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withWheelRadius(SwerveConstants.wheelRadius)
            .withFeedforward(
                SimpleMotorFeedforward(
                    SwerveConstants.DRIVE_S,
                    SwerveConstants.DRIVE_V,
                    SwerveConstants.DRIVE_A
                )
            )
            .withClosedLoopController(SwerveConstants.DRIVE_P, SwerveConstants.DRIVE_I, SwerveConstants.DRIVE_D, SmartMotorController.ClosedLoopControllerSlot.SLOT_0)
            .withGearing(SwerveConstants.driveGearing)
            .withOpenLoopRampRate(Seconds.of(0.05))
            .withClosedLoopRampRate(Seconds.of(0.0))
            .withMotorInverted(true)
            .withStatorCurrentLimit(SwerveConstants.driveCurrentLimit)
            .withTelemetry("$moduleName Drive Motor", TelemetryConstants.DRIVETRAIN_DRIVE_TELEMETRY)

        val angleConfig = SmartMotorControllerConfig(this)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withGearing(SwerveConstants.angleGearing)
            .withClosedLoopController(SwerveConstants.ANGLE_P, 0.0, SwerveConstants.ANGLE_D)
            .withContinuousWrapping(-Radians.of(PI), Radians.of(PI))
            .withStatorCurrentLimit(SwerveConstants.angleCurrentLimit)
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withClosedLoopRampRate(Seconds.of(0.0))
            .withMotorInverted(true)
            .withTelemetry("$moduleName Angle Motor", TelemetryConstants.DRIVETRAIN_ANGLE_TELEMETRY)

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

    private var driveOpenLoop = false

    private val gyroConfig = MountPoseConfigs().withMountPoseRoll(Degrees.of(-0.4267374873161316))
        .withMountPoseYaw(Degrees.of(178.08946228027344)).withMountPosePitch(Degrees.of(-0.06843218952417374))
    private val gyro = Pigeon2(13)
    private val gyroYaw = gyro.yaw.asSupplier()
    private val gyroPitch = gyro.pitch.asSupplier()
    private val gyroRoll = gyro.roll.asSupplier()
    private val gyroYawVel = gyro.angularVelocityZWorld.asSupplier()
    private val gyroRollVel = gyro.angularVelocityXWorld.asSupplier()
    private val gyroPitchVel = gyro.angularVelocityYWorld.asSupplier()

    private val kinematics = SwerveDriveKinematics(*SwerveConstants.moduleTranslations)
    private var moduleStates = Array(4) { modules[it].state }

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d(gyroYaw.get()),
        Array(4) { modules[it].position },
        Pose2d(Translation2d.kZero, Rotation2d(gyroYaw.get()))
    )

    private val limelight = Limelight("limelight-green")
    private val limelightPoseEstimatorMT2 =
        limelight.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2)
    private var fed = false

    private val field = Field2d()

    init {
        SmartDashboard.putData(field)

        defaultCommand = JoystickDrive(Input::getFieldOriented)

        gyro.configurator.apply(gyroConfig)
        poseEstimator.setVisionMeasurementStdDevs(VisionConstants.visionStdv)

        DogLog.log("Drivetrain/SwerveModuleStates/Setpoints", Array(4) { SwerveModuleState() })
        DogLog.log("Drivetrain/SwerveModuleStates/Optimized Setpoints", Array(4) { SwerveModuleState() })

    }

    override fun periodic() {
        modules.forEach { it.updateTelemetry() }
        modules.forEach { it.seedAzimuthEncoder() }

        updatePoseEstimator()

        if (!fed) {
            driveRobotRelative(ChassisSpeeds())
        }

        fed = false

        DogLog.log("Drivetrain/RobotPose", getPose())
    }

    private fun updatePoseEstimator() {
        poseEstimator.update(Rotation2d(gyroYaw.get()), getModulePositions())

        moduleStates = getModuleStates()
        DogLog.log("Drivetrain/SwerveModuleStates/Measured", moduleStates)

        val chassisSpeeds = getChassisSpeeds()
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
        fed = true

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
            module.setSwerveModuleState(states[index])
        }
        return Array(4) { modules[it].config.getOptimizedState(states[it]) }
    }

    private fun getModulePositions(): Array<SwerveModulePosition> {
        return Array(4) { modules[it].position }
    }

    private fun getModuleStates(): Array<SwerveModuleState> {
        return Array(4) { modules[it].state }
    }

    private fun updateVision() {
        limelight.settings.withRobotOrientation(
            Orientation3d(
                Rotation3d(gyroRoll.get(), gyroPitch.get(), gyroYaw.get()),
                gyroYawVel.get(),
                gyroPitchVel.get(),
                gyroRollVel.get()
            )
        )

        val estimatedPose = limelightPoseEstimatorMT2.poseEstimate
        if (estimatedPose.isEmpty) {
            return
        }

        DogLog.log("Drivetrain/EstimatedPose", estimatedPose.get().pose)
        if (estimatedPose.get().pose.toPose2d().translation == Translation2d.kZero) {
            return
        }
        poseEstimator.addVisionMeasurement(estimatedPose.get().pose.toPose2d(), Timer.getFPGATimestamp())
    }

    fun stopDrivePID() {
        modules.forEach {
            it.driveMotorController.setFeedback(0.0, 0.0, 0.0)
        }
    }

    fun startDrivePID() {
        modules.forEach {
            it.driveMotorController.setFeedback(SwerveConstants.DRIVE_P, 0.0, SwerveConstants.DRIVE_D)
        }
    }

    fun getChassisSpeeds(): ChassisSpeeds {
        return kinematics.toChassisSpeeds(*moduleStates)
    }

    fun getPose(): Pose2d {
        return poseEstimator.estimatedPosition
    }

    fun setPose(pose: Pose2d) {
        poseEstimator.resetTranslation(pose.translation)
        setRotation(pose.rotation)
    }

    fun setRotation(rotation: Rotation2d) {
        gyro.setYaw(rotation.measure)
    }

    fun setCurrentLimit(current: Current) {
        modules.forEach {
            it.driveMotorController.setStatorCurrentLimit(current)
        }
    }

    fun runFFCharacterization(output: Double) {
        modules.forEach {
            it.driveMotorController.voltage = Volts.of(output)
            it.azimuthMotorController.setPosition(Rotations.of(0.0))
        }
    }

    fun getFFCharacterizationVelocity(): Double {
        var output = 0.0
        modules.forEach {
            output += it.driveMotorController.mechanismVelocity.`in`(RotationsPerSecond)
        }
        return output / 4.0
    }

    /* Game Specific */
    fun rotationTo(other: Translation2d): Rotation2d {
        val driveTranslation = getPose().translation
        return Rotation2d(atan2(other.y - driveTranslation.y, other.x - driveTranslation.x))
    }

    fun distanceTo(translation2d: Translation2d): Distance {
        return Meters.of(getPose().translation.getDistance(translation2d))
    }

    fun rotationToClosestTranslation(vararg translations: Translation2d): Rotation2d {
        return rotationTo(getPose().translation.nearest(translations.toSet()))
    }

    fun getClosestRotation(vararg rotations: Rotation2d): Rotation2d {
        var closest = 1.0
        var rotation = rotations[0]
        rotations.forEach {
            if (abs(it.rotations - getPose().rotation.rotations) % 1.0 < closest) {
                closest = abs(rotation.rotations - getPose().rotation.rotations)
                rotation = it
            }
        }

        return rotation
    }

    fun distanceToClosest(vararg translations: Translation2d): Distance {
        return distanceTo(getPose().translation.nearest(translations.toSet()))
    }

    fun getIsMoving(): Boolean {
        val chassisSpeeds = getChassisSpeeds()
        return MetersPerSecond.of(
            hypot(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond
            )
        ) > SwerveConstants.movingThreshold
    }
}