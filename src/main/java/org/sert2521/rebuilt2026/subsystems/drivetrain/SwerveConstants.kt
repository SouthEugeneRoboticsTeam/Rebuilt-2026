package org.sert2521.rebuilt2026.subsystems.drivetrain

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Distance
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object SwerveConstants {
    val driveGearing = MechanismGearing(GearBox.fromReductionStages(6.75))
    val angleGearing = MechanismGearing(GearBox.fromReductionStages(150.0/7.0))

    val wheelRadius: Distance = Units.Inches.of(2.0)
    const val WHEEL_COF = 1.54

    val moduleNames = arrayOf("Front Left", "Front Right", "Back Left", "Back Right")
    val moduleTranslations = arrayOf(
        Translation2d(Units.Meters.of(0.552450), Units.Meters.of(0.552450)),
        Translation2d(Units.Meters.of(0.552450), -Units.Meters.of(0.552450)),
        Translation2d(-Units.Meters.of(0.552450), Units.Meters.of(0.552450)),
        Translation2d(-Units.Meters.of(0.552450), -Units.Meters.of(0.552450))
    )
    val moduleZeroRotations = arrayOf(
        Rotations.of(1.0-0.927734),
        Rotations.of(1.0-0.652832),
        Rotations.of(0.001465),
        Rotations.of(-0.143066)
    )

    val encoderIDs = arrayOf(1, 2, 3, 4)
    val driveIDs = arrayOf(5, 6, 7, 8)
    val angleIDs = arrayOf(9, 10, 11, 12)

    // TODO: Tune
    const val DRIVE_P = 0.0
    const val DRIVE_D = 0.0
    const val DRIVE_S = 0.0
    const val DRIVE_V = 0.0
    val driveCurrentLimit = Units.Amps.of(40.0)

    // TODO: Tune
    const val ANGLE_P = 0.0
    const val ANGLE_D = 0.0
    val angleCurrentLimit = Units.Amps.of(40.0)

    // TODO: Tune
    const val AUTO_TRANSLATION_P = 0.0
    const val AUTO_TRANSLATION_I = 0.0
    const val AUTO_TRANSLATION_D = 0.0

    // TODO: Tune
    const val AUTO_HEADING_P = 0.0
    const val AUTO_HEADING_I = 0.0
    const val AUTO_HEADING_D = 0.0

    // TODO: Tune
    const val VISION_HEADING_P = 0.0
    const val VISION_HEADING_D = 0.0

    val maxSpeed = Units.MetersPerSecond.of(4.571)

    const val SYS_ID_FF_RAMP_RATE = 1.0

    const val DRIVE_SPEED = 4.5

    const val ROT_SPEED = 4.0
}