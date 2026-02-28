package org.sert2521.rebuilt2026.subsystems.shooter

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage

// Stands for "Hooded Shooter Goal"
// Allows you to encode all targets in one variable
data class HSGoal(val firstFlywheelsSpeed: AngularVelocity, val secondFlywheelsSpeed: AngularVelocity, val rollersVoltage: Voltage)