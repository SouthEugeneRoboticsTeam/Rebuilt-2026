package org.sert2521.rebuilt2026.util

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity

// Stands for "Hooded Shooter Goal"
// Allows you to encode both targets in one variable
data class HSGoal(val hoodGoalRotations: Double, val flywheelGoalRPM: Double)
