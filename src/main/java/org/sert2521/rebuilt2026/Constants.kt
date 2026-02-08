package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 1
    const val GRINTAKE_WRIST_MOTOR_ID = 2

    const val KICKER_MOTOR_ID = 3
    const val INDEXER_MOTOR_ID = 4

    const val SHOOTER_LEADER_MOTOR_ID = 5
    const val SHOOTER_FOLLOWER_MOTOR_ID = 6
    const val SHOOTER_ROLLER_MOTOR_ID = 7

    const val INDEXER_BEAM_BREAK_ID = 0
}

object GrintakeConstants {
    const val rollerP = 0.0
    const val rollerD = 0.0
    const val wristP = 0.0
    const val wristD = 0.0

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0  // FIXME: add reduction stages
        )
    )
    val wristGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0  // FIXEM: add reduction stages
        )
    )

    val stowPosition = Degrees.of(0.0)
    val intakePosition = Degrees.of(0.0)

    val intakeSpeed = 0.2
    val reverseSpeed = -0.2

    val hardMin = Rotations.of(0.0)
    val hardMax = Rotations.of(0.0)
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )
    val kickerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )
    const val MAIN_DEFAULT = 0.0
    const val KICKER_DEFAULT = 0.0

    const val MAIN_INDEXING = 0.0
    const val KICKER_INDEXING = 0.0

    const val MAIN_KICKING = 0.0
    const val KICKER_KICKING = 0.0
    const val KICK_TIME = 0.0

    const val MAIN_REVERSE = 0.0
    const val KICKER_REVERSE = 0.0
}

object HoodedShooterConstants {
    const val P = 0.0
    const val D = 0.0
    const val S = 0.0
    const val V = 0.0
    const val A = 0.0

    val shooterGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0  // FIXME: Add reduction stages.
        )
    )

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0  // FIXME: Add reducton stages.
        )
    )

    val shootTarget = RPM.of(0.0)

    val shootRollerDutyCycle = 0.0
}

object OtherConstsants {
    val targetVisionPose = Pose2d()
}