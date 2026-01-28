package org.sert2521.rebuilt2026

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Rotations
import org.sert2521.rebuilt2026.util.HSGoal
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 1
    const val GRINTAKE_WRIST_MOTOR_ID = 2
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

}

object HoodedShooterConstants {

}

object OtherConstants {

}
