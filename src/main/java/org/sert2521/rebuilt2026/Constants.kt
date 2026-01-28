package org.sert2521.rebuilt2026

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.sert2521.rebuilt2026.util.HSGoal
import yams.gearing.GearBox
import yams.gearing.MechanismGearing

object GrintakeConstants {

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

object ElectronicIDs {

}
