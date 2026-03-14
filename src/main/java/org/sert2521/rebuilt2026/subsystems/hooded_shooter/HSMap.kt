package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.Distance
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain

object HSMap {
    private val blueAllianceHub = Translation2d()
    private val redAllianceHub = Translation2d()

    private val dataPoints = arrayOf(
        HSMapDatapoint(
            Meters.of(0.0),
            HSGoal(
                RPM.of(2500.0),
                RPM.of(2300.0),
                RPM.of()
            )
        )
    )

    private fun interpolateWithDistance(distance: Distance): HSGoal{
        var lastDatapoint = dataPoints[0]
        var nextDatapoint = dataPoints[0]

        // Don't worry, Genius Benji Turner is at it again (this saves like 5 loops of cpu time but whatever)
        run loop@{dataPoints.forEach {
            if (it.distance > distance) {
                nextDatapoint = it
                return@loop
            } else {
                lastDatapoint = it
            }
        }}

        val p = MathUtil.inverseInterpolate(
            lastDatapoint.distance.`in`(Meters),
            nextDatapoint.distance.`in`(Meters),
            distance.`in`(Meters)
        )

        return HSGoal(
            RPM.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.firstFlywheelsSpeed.`in`(RPM),
                nextDatapoint.hsGoal.firstFlywheelsSpeed.`in`(RPM),
                p
            )),
            RPM.of(MathUtil.interpolate(
                lastDatapoint.hsGoal.secondFlywheelsSpeed.`in`(RPM),
                nextDatapoint.hsGoal.secondFlywheelsSpeed.`in`(RPM),
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

    fun getGoal(): HSGoal{
        return interpolateWithDistance(Drivetrain.getDistanceToHub())
    }
}