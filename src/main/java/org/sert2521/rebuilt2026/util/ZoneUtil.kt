package org.sert2521.rebuilt2026.util

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import javax.print.attribute.standard.MediaSize.Other

object ZoneUtil {
    enum class Zone {
        ALLIANCE,
        NEUTRAL,
        OPPONENT
    }

    fun getCurrentZone(): Zone {
        return when {
            Drivetrain.getPose().x in OtherConstsants.blueBoundary.`in`(Meters)..OtherConstsants.redBoundary.`in`(Meters) -> Zone.NEUTRAL
            (Drivetrain.getPose().x in OtherConstsants.redBoundary.`in`(Meters)..Double.MAX_VALUE) xor allianceIsBlue()   -> Zone.ALLIANCE
            else                                                                                                                -> Zone.OPPONENT
        }
    }

    fun isLeft(): Boolean {
        return allianceIsBlue().xor(Drivetrain.getPose().y < OtherConstsants.midline.`in`(Meters))
    }

    fun isDepot(): Boolean {
        return OtherConstsants.blueDepotArea.intersects(Drivetrain.getPose().translation)
                || OtherConstsants.redDepotArea.intersects(Drivetrain.getPose().translation)
    }

    fun getShallows(): Array<Rotation2d> {
        return if (isLeft()) {
            OtherConstsants.shallowsCW
        } else {
            OtherConstsants.shallowsCCW
        }
    }

    fun getTower(): Rotation2d {
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

    fun getDriverAssistRotation():Rotation2d {
        return if (allianceIsBlue()) {
            Rotation2d.k180deg
        } else {
            Rotation2d.kZero
        }
    }

    fun getScoringRotationTarget():Rotation2d {
        return when (getCurrentZone()) {
            Zone.ALLIANCE -> Drivetrain.rotationTo(OtherConstsants.currentHub())
                .rotateBy(Rotation2d.k180deg)

            Zone.NEUTRAL -> Drivetrain.rotationToClosestTranslation(*OtherConstsants.passTargetsClose())
                .rotateBy(Rotation2d.k180deg)

            Zone.OPPONENT -> Drivetrain.rotationToClosestTranslation(*OtherConstsants.passTargetsClose())
                .rotateBy(Rotation2d.k180deg)
        }
    }

    fun getUtilRotationTarget():Rotation2d {
        return when (getCurrentZone()) {
            Zone.ALLIANCE -> getTower()
            Zone.NEUTRAL, Zone.OPPONENT -> Drivetrain.getClosestRotation(*getShallows())
        }
    }
}