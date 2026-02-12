package org.sert2521.rebuilt2026.commands


import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import java.util.*


class DrivetrainFeedforwardSysId : SequentialCommandGroup(){
    val velocitySamples: MutableList<Double> = LinkedList()
    val voltageSamples: MutableList<Double> = LinkedList()
    val timer = Timer()

    init {
        addCommands(
            Commands.runOnce(
                {
                    velocitySamples.clear()
                    voltageSamples.clear()
                    Drivetrain.stopDrivePID()
                }
            ),
            Commands.run(
                {
                    Drivetrain.runFFCharacterization(0.0)
                },
                Drivetrain
            ).withTimeout(1.0),
            Commands.runOnce(timer::restart),
            Commands.run(
                {
                    val voltage: Double = timer.get() * SwerveConstants.SYS_ID_FF_RAMP_RATE
                    Drivetrain.runFFCharacterization(voltage)
                    velocitySamples.add(Drivetrain.getFFCharacterizationVelocity())
                    voltageSamples.add(voltage)
                },
                Drivetrain
            )
        )
    }
}
