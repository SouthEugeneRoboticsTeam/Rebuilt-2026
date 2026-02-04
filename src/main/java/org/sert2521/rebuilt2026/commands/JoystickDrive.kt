package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import kotlin.math.pow

class JoystickDrive(private val fieldOriented:Boolean = false) : Command() {
    private var targetChassisSpeeds = ChassisSpeeds()

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        Drivetrain.stopDrivePID()
    }

    override fun execute() {
        targetChassisSpeeds = ChassisSpeeds(
            Input.getLeftX().pow(3) * SwerveConstants.DRIVE_SPEED,
            Input.getLeftY().pow(3) * SwerveConstants.DRIVE_SPEED,
            Input.getRightRot().pow(3) * SwerveConstants.ROT_SPEED
        )

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
}