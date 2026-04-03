package org.sert2521.rebuilt2026.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import java.util.function.BooleanSupplier
import kotlin.math.atan2

class TankDriveSortOf(fieldOriented:BooleanSupplier):VisionRotationDrive(fieldOriented,
    {
        val fieldOrientedSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(Drivetrain.getChassisSpeeds(), Drivetrain.getPose().rotation)
        Rotation2d(atan2(fieldOrientedSpeeds.vyMetersPerSecond, fieldOrientedSpeeds.vxMetersPerSecond))
    }
)