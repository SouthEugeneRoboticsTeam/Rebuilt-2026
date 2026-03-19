package org.sert2521.rebuilt2026.util

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain

object HSMap {
    private fun interpolateWithDistance(distance: Distance, dataset: Array<HSMapDatapoint>): HSGoal {
        var lastDatapoint = dataset[0]
        var nextDatapoint = dataset[0]

        for (point in dataset){
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
            RPM.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.firstFlywheelSpeed.`in`(RPM),
                nextDatapoint.hsGoal.firstFlywheelSpeed.`in`(RPM),
                p
            )),
            RPM.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.secondFlywheelSpeed.`in`(RPM),
                nextDatapoint.hsGoal.secondFlywheelSpeed.`in`(RPM),
                p
            )),
            RPM.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.rollerVelocity.`in`(RPM),
                nextDatapoint.hsGoal.rollerVelocity.`in`(RPM),
                p
            )),
            Degrees.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.hoodAngle.`in`(Degrees),
                nextDatapoint.hsGoal.hoodAngle.`in`(Degrees),
                p
            ))
        )
    }

    fun getGoalHub(): HSGoal {
        return interpolateWithDistance(Drivetrain.distanceTo(OtherConstsants.currentHub), OtherConstsants.dataHub)
    }

    fun getGoalPass(): HSGoal {
        return interpolateWithDistance(Drivetrain.distanceToClosest(*OtherConstsants.currentBumps), OtherConstsants.dataPass)
    }
}