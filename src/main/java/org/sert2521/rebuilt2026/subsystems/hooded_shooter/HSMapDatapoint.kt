package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.Distance

class HSMapDatapoint{
    val distance:Distance
    val hsGoal:HSGoal

    constructor(distance:Distance, hsGoal:HSGoal){
        this.distance = distance
        this.hsGoal = hsGoal
    }

    constructor(distance:Double, primary:Double, secondary:Double, rollers:Double, hood:Double){
        this.distance = Meters.of(distance)
        this.hsGoal = HSGoal(RPM.of(primary), RPM.of(secondary), RPM.of(rollers), Degrees.of(hood))
    }
}