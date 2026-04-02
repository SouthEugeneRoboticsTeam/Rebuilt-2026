package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.JoystickSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import java.util.function.BooleanSupplier
import java.util.function.Supplier
import kotlin.math.*

class VisionRotationDrive(private val fieldOriented: BooleanSupplier, private val rotationTarget: Supplier<Rotation2d>) : Command() {
    companion object{
        private val rotationPID = PIDController(SwerveConstants.VISION_HEADING_P, 0.0, SwerveConstants.VISION_HEADING_D)
    }

    private var targetChassisSpeeds = ChassisSpeeds()

    private val rateLimiter = JoystickDrive.rateLimiter

    init {
        addRequirements(Drivetrain)
        SmartDashboard.putData("Rotation PID", rotationPID)
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        rotationPID.reset()
    }

    override fun execute() {
        val theta = atan2(Input.getLeftY(), Input.getLeftX())
        val mag = MathUtil.applyDeadband(hypot(Input.getLeftY(), Input.getLeftX()), 0.1)
        val ratedMag = rateLimiter.calculate(min(mag.pow(3), Input.maxSpeed()))

        targetChassisSpeeds = ChassisSpeeds(
            sin(theta) * SwerveConstants.DRIVE_SPEED * ratedMag,
            cos(theta) * SwerveConstants.DRIVE_SPEED * ratedMag,
            rotationPID.calculate(Drivetrain.getPose().rotation.radians, rotationTarget.get().radians)
        )

        Drivetrain.driveRobotRelative(
            if (fieldOriented.asBoolean) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    targetChassisSpeeds,
                    Drivetrain.getPose().rotation.minus(Input.getRotOffset())
                )
            } else {
                targetChassisSpeeds
            }
        )
    }
}