package org.sert2521.rebuilt2026.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.sert2521.rebuilt2026.util.HSMap
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Flywheel
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Hood
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Roller

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
            Roller.stop(),
            Hood.stop()
        )
    }

    fun revStatic(): Command {
        return revAndTrackHub()
    }

    fun revAndTrackPass(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { this.currentGoal.firstFlywheelSpeed },
            Roller.setVelocity { this.currentGoal.rollerVelocity },
            Hood.setPosition { this.currentGoal.hoodAngle }
        ).alongWith(Commands.run(::updateGoalPass))
    }

    fun revAndTrackHub(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { this.currentGoal.firstFlywheelSpeed },
            Roller.setVelocity { this.currentGoal.rollerVelocity },
            Hood.setPosition { this.currentGoal.hoodAngle }
        ).alongWith(Commands.run(::updateGoalHub))
    }

    private fun sustainAndTrack(): Command {
        return Commands.parallel(
            Flywheel.setVelocity { this.currentGoal.secondFlywheelSpeed } ,
            Roller.setVelocity { this.currentGoal.rollerVelocity },
            Hood.setPosition { this.currentGoal.hoodAngle }
        ).alongWith(Commands.run(::updateGoalHub))
    }



    fun shoot(): Command {
        return revAndTrackHub().withTimeout(0.5).andThen(sustainAndTrack())
            .alongWith(Commands.run(::updateGoalHub))
    }
}