package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 21
    const val GRINTAKE_WRIST_MOTOR_ID = 22

    const val INDEXER_MOTOR_ID = 31
    const val KICKER_MOTOR_ID = 32

    const val FLYWHEEL_LEFT_ID = 41
    const val FLYWHEEL_RIGHT_ID = 42
    const val SHOOTER_ROLLER_MOTOR_ID = 43

    const val INDEXER_BEAM_BREAK_ID = 1
}

object GrintakeConstants {
    const val WRIST_P = 6.4
    const val WRIST_D = 0.0

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            24.0/12.0,
            36.0/24.0
        )
    )
    val wristGearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            54.0/24.0,
            56.0/24.0
        )
    )

    val stowPosition = Rotations.of(0.07)
    val intakePosition = Rotations.of(0.42) - Degrees.of(1.0)

    const val INTAKE_SPEED = 1.0
    const val REVERSE_SPEED = 0.0

    const val REZERO_SPEED = -0.2
    val reZeroThreshold = Amps.of(20.0)
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            4.0,
            28.0/24.0
        )
    )
    val kickerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0
        )
    )

    const val MAIN_DEFAULT = 0.0
    const val KICKER_DEFAULT = 0.0

    const val MAIN_INDEXING = 0.4
    const val KICKER_INDEXING = -0.8

    const val MAIN_KICKING = 0.7
    const val KICKER_KICKING = 0.9
    const val KICK_TIME = 0.0

    const val MAIN_REVERSE = -0.4
    const val KICKER_REVERSE = -0.4

    const val PULSE_SHOOT_TIME = 100.0
    const val PULSE_DELAY_TIME = 0.0
}

object HoodedShooterConstants {
    const val P = 0.02
    const val D = 0.001
    const val S = 0.0
    const val V = 0.152
    const val A = 0.0
    const val Q = 0.07

    val shooterGearing = MechanismGearing(
        GearBox.fromReductionStages(
            24.0 / 20.0
        )
    )

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )


    val primaryShootFlywheel = RPM.of(2500.0)
    val secondaryShootFlywheel = RPM.of(2300.0)
    val shootRollerDutyCycle = Volts.of(6.0)
}

object OtherConstsants {
    val blueHubTranslation = Translation2d(4.620755195617676,4.036807537078857)
}