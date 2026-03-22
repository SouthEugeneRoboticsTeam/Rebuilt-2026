package org.sert2521.rebuilt2026.util

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance

class HSMapDatapoint{
    val distance:Distance
    val hsGoal: HSGoal

    constructor(hsGoal: HSGoal,distance:Distance){
        this.distance = distance
        this.hsGoal = hsGoal
    }

    constructor(distance:Double, primary:Double, hood:Double){
        this.distance = Meters.of(distance)
        this.hsGoal = HSGoal(RPM.of(primary),Rotations.of(hood))
    }

    constructor(primary:AngularVelocity, hood: Angle,distance:Distance){
        this.distance = distance
        this.hsGoal = HSGoal(primary, hood)
    }
}