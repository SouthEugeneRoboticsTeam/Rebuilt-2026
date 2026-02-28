package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

class VisionRotationDrive(private val fieldOriented: Boolean = true) : Command() {
    private var targetChassisSpeeds = ChassisSpeeds()
    private val rotationPID = PIDController(SwerveConstants.VISION_HEADING_P, 0.0, SwerveConstants.VISION_HEADING_D)


    init {
        addRequirements(Drivetrain)
        SmartDashboard.putData("Rotation PID", rotationPID)
        rotationPID.enableContinuousInput(-PI, PI)
    }

    override fun initialize() {
        Drivetrain.stopDrivePID()
    }

    override fun execute() {
        val theta = atan2(Input.getLeftY(), Input.getLeftX())
        val mag = hypot(Input.getLeftY(), Input.getLeftX())
        if (mag<1.0){
            val corrMag = mag.pow(3)
            targetChassisSpeeds = ChassisSpeeds(
                sin(theta) * corrMag * SwerveConstants.DRIVE_SPEED,
                cos(theta) * corrMag * SwerveConstants.DRIVE_SPEED,
                rotationPID.calculate(Drivetrain.getPose().rotation.radians, Drivetrain.getRotationToHub().radians)
            )
        } else {
            val y = sin(theta)
            val x = cos(theta)
            targetChassisSpeeds = ChassisSpeeds(
                y * SwerveConstants.DRIVE_SPEED,
                x * SwerveConstants.DRIVE_SPEED,
                rotationPID.calculate(Drivetrain.getPose().rotation.radians, Drivetrain.getRotationToHub().radians)
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