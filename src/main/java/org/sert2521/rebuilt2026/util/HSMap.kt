package org.sert2521.rebuilt2026.util

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import kotlin.math.hypot
import kotlin.math.min

object HSMap {
    private fun interpolateWithDistance(distance: Distance, dataset: Array<HSMapDatapoint>): HSGoal {
        var lastDatapoint = dataset.first()
        var nextDatapoint = dataset.last()

        for (point in dataset) {
            // Remember that the dataset order should be in increasing distance
            if (point.distance > distance) {
                nextDatapoint = point
                break
            } else {
                lastDatapoint = point
            }
        }

        val p = MathUtil.inverseInterpolate(
            lastDatapoint.distance.`in`(Meters),
            nextDatapoint.distance.`in`(Meters),
            distance.`in`(Meters)
        )

        return HSGoal(
            RPM.of(
                min(
                    MathUtil.interpolate(
                        lastDatapoint.hsGoal.flywheelSpeed.`in`(RPM),
                        nextDatapoint.hsGoal.flywheelSpeed.`in`(RPM),
                        p
                    ), nextDatapoint.hsGoal.flywheelSpeed.`in`(RPM)
                )
            ),
            Rotations.of(
                min(
                    MathUtil.interpolate(
                        lastDatapoint.hsGoal.hoodAngle.`in`(Rotations),
                        nextDatapoint.hsGoal.hoodAngle.`in`(Rotations),
                        p
                    ), nextDatapoint.hsGoal.hoodAngle.`in`(Rotations)
                )
            ),
        )
    }

    fun getGoalHub(): HSGoal {
        return interpolateWithDistance(Drivetrain.distanceTo(OtherConstsants.currentHub()), OtherConstsants.dataHub)
    }

    fun getGoalHubWithMovement(): HSGoal {
        return interpolateWithDistance(
            Drivetrain.distanceTo(OtherConstsants.currentHub()) +
                    Meters.of(
                        hypot(
                            Drivetrain.getChassisSpeeds().vxMetersPerSecond,
                            Drivetrain.getChassisSpeeds().vyMetersPerSecond
                        )
                                * OtherConstsants.distanceSpeedAdjustment.`in`(Seconds)
                    ),
            OtherConstsants.dataHub
        )
    }

    fun getGoalPass(): HSGoal {
        return interpolateWithDistance(
            Drivetrain.distanceToClosest(*OtherConstsants.currentBumps()),
            OtherConstsants.dataPass
        )
    }
}