package org.sert2521.rebuilt2026.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import org.sert2521.rebuilt2026.OtherConstsants
import kotlin.jvm.optionals.getOrElse

fun Translation2d.flipWidth(): Translation2d {
    return Translation2d(this.x, OtherConstsants.fieldWidth.`in`(Units.Meters) - this.y)
}

fun Translation2d.flipAlliance(): Translation2d {
    return Translation2d(OtherConstsants.fieldLength.`in`(Units.Meters) - this.x, this.y)
}

fun allianceIsBlue(): Boolean {
    return DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Blue
}