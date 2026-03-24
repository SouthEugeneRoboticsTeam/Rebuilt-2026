package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

class JoystickDrive(private val fieldOriented: Boolean = true) : Command() {
    private var targetChassisSpeeds = ChassisSpeeds()

    private val rateLimiter = SlewRateLimiter(1.0/SwerveConstants.timeToFullSpeed.`in`(Seconds), -99999.9, 0.0)

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
    }

    override fun execute() {
        val theta = atan2(Input.getLeftY(), Input.getLeftX())
        val mag = MathUtil.applyDeadband(hypot(Input.getLeftY(), Input.getLeftX()), 0.1)
        if (mag<1.0){
            val corrMag = mag.pow(3)
            val ratedMag = rateLimiter.calculate(corrMag)
            targetChassisSpeeds = ChassisSpeeds(
                sin(theta) * ratedMag * SwerveConstants.DRIVE_SPEED,
                cos(theta) * ratedMag * SwerveConstants.DRIVE_SPEED,
                Input.getRightRot().pow(3) * SwerveConstants.ROT_SPEED
            )
        } else {
            val y = sin(theta)
            val x = cos(theta)
            val ratedMag = rateLimiter.calculate(1.0)
            targetChassisSpeeds = ChassisSpeeds(
                y * SwerveConstants.DRIVE_SPEED * ratedMag,
                x * SwerveConstants.DRIVE_SPEED * ratedMag,
                Input.getRightRot().pow(3) * SwerveConstants.ROT_SPEED
            )
        }


        if (fieldOriented) {
            Drivetrain.driveRobotRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    targetChassisSpeeds,
                    Drivetrain.getPose().rotation.minus(Input.getRotOffset())
                )
            )
        } else {
            Drivetrain.driveRobotRelative(targetChassisSpeeds)
        }
    }

    override fun end(interrupted: Boolean) {
    }
}