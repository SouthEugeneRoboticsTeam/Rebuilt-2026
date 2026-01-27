package org.sert2521.rebuilt2026.commands

import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain

class JoystickDrive : Command() {
    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        // Drivetrain.stopPID()
    }

    override fun execute() {
        // TODO: Put drive code
    }
}