package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity

// Stands for "Hooded Shooter Goal"
// Allows you to encode all targets in one variable
data class HSGoal(val firstFlywheelsSpeed: AngularVelocity, val secondFlywheelsSpeed: AngularVelocity, val rollerVelocity: AngularVelocity, val hoodAngle: Angle)