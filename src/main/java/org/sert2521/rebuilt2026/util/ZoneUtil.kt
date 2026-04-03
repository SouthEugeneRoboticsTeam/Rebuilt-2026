package org.sert2521.rebuilt2026.util

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain

object ZoneUtil {
    enum class Zone {
        ALLIANCE,
        NEUTRAL,
        OPPONENT
    }

    fun getCurrentZone():Zone{
        return if (allianceIsBlue()){
            when (Drivetrain.getPose().x) {
                in Double.MIN_VALUE..OtherConstsants.blueBoundary.`in`(Meters) -> Zone.ALLIANCE
                in OtherConstsants.blueBoundary.`in`(Meters)..OtherConstsants.redBoundary.`in`(Meters) -> Zone.NEUTRAL
                in OtherConstsants.redBoundary.`in`(Meters)..Double.MAX_VALUE -> Zone.OPPONENT
                else -> Zone.ALLIANCE
            }
        } else {
            when (Drivetrain.getPose().x) {
                in Double.MIN_VALUE..OtherConstsants.blueBoundary.`in`(Meters) -> Zone.OPPONENT
                in OtherConstsants.blueBoundary.`in`(Meters)..OtherConstsants.redBoundary.`in`(Meters) -> Zone.NEUTRAL
                in OtherConstsants.redBoundary.`in`(Meters)..Double.MAX_VALUE -> Zone.ALLIANCE
                else -> Zone.ALLIANCE
            }
        }
    }

    fun isLeft(): Boolean {
        return allianceIsBlue().xor(Drivetrain.getPose().y < OtherConstsants.midline.`in`(Meters))
    }

    fun getShallows():Array<Rotation2d> {
        return if (isLeft()){
            OtherConstsants.shallowsCW
        } else {
            OtherConstsants.shallowsCCW
        }
    }

    fun getTower():Rotation2d {
        return if (allianceIsBlue()) {
            if (isLeft()) {
                OtherConstsants.fortyFives[3]
            } else {
                OtherConstsants.fortyFives[0]
            }
        } else {
            if (isLeft()) {
                OtherConstsants.fortyFives[1]
            } else {
                OtherConstsants.fortyFives[2]
            }
        }
    }

    fun getDriverAssistLine(): Distance {
        return when (getCurrentZone()) {
            Zone.ALLIANCE -> if (allianceIsBlue()) {
                OtherConstsants.driverAssistLineWall
            } else {
                OtherConstsants.fieldLength - OtherConstsants.driverAssistLineWall
            }

            Zone.NEUTRAL -> if (allianceIsBlue()) {
                OtherConstsants.driverAssistLineHub
            } else {
                OtherConstsants.fieldLength - OtherConstsants.driverAssistLineHub
            }

            Zone.OPPONENT -> if (allianceIsBlue()) {
                OtherConstsants.fieldLength - OtherConstsants.driverAssistLineWall
            } else {
                OtherConstsants.driverAssistLineWall
            }
        }
    }
}