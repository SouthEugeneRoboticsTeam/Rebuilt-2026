package org.sert2521.rebuilt2026.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.rebuilt2026.Input
import org.sert2521.rebuilt2026.OtherConstsants
import org.sert2521.rebuilt2026.ShooterConstants
import org.sert2521.rebuilt2026.util.HSMap
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Flywheel
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Hood

object HoodedShooterCommands {
    private var currentGoal = HSMap.getGoalHub()

    fun updateGoalHub() {
        currentGoal = HSMap.getGoalHub()
    }

    fun updateGoalPass() {
        currentGoal = HSMap.getGoalPass()
    }

    fun stop(): Command {
        return Commands.parallel(
            Flywheel.stop(),
            Hood.stop()
        )
    }

    fun liveTuning(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { OtherConstsants.flywheelLiveSetpoint },
            Hood.setPosition { ShooterConstants.hoodMax * Input.getGunnerSlider() }
        )
    }

    fun revAndTrackPass(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { this.currentGoal.flywheelSpeed },
            Hood.setPosition { this.currentGoal.hoodAngle }
        ).alongWith(Commands.run(::updateGoalPass))
    }

    fun revAndTrackHub(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { this.currentGoal.flywheelSpeed },
            Hood.setPosition { this.currentGoal.hoodAngle }
        ).alongWith(Commands.run(::updateGoalHub))
    }
}