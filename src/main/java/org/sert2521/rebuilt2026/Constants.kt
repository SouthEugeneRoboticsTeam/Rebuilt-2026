package org.sert2521.rebuilt2026

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.sert2521.rebuilt2026.util.HSGoal
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 1
    const val GRINTAKE_WRIST_MOTOR_ID = 2

    const val KICKER_MOTOR_ID = 3
    const val INDEXER_MOTOR_ID = 4
}

object GrintakeConstants {
    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            //add reduction stages
        )
    )
    val wristGearing = MechanismGearing(
        GearBox.fromReductionStages(
            //add reduction stages
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
    val IndexerGearing = MechanismGearing(
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

}

object OtherConstants {

}