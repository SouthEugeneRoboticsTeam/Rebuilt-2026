package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import java.util.function.BooleanSupplier
import kotlin.math.*

class JoystickDrive(private val fieldOriented: BooleanSupplier) : Command() {
    companion object {
        val rateLimiter = SlewRateLimiter(1.0 / SwerveConstants.timeToFullSpeed.`in`(Seconds), -99999.9, 0.0)
    }

    private var targetChassisSpeeds = ChassisSpeeds()


    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
    }

    override fun execute() {
        val theta = atan2(Input.getLeftY(), Input.getLeftX())
        val mag = MathUtil.applyDeadband(hypot(Input.getLeftY(), Input.getLeftX()), 0.1)
        val ratedMag = rateLimiter.calculate(min(mag.pow(3), Input.maxSpeed()))

        targetChassisSpeeds = ChassisSpeeds(
            sin(theta) * SwerveConstants.DRIVE_SPEED * ratedMag,
            cos(theta) * SwerveConstants.DRIVE_SPEED * ratedMag,
            Input.getRightRot().pow(3) * SwerveConstants.ROT_SPEED
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