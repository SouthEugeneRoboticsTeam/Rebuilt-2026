package org.sert2521.rebuilt2026

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.idle
import org.sert2521.rebuilt2026.subsystems.Grintake
import org.sert2521.rebuilt2026.subsystems.HoodedShooter
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import kotlin.jvm.optionals.getOrElse

object Autos {
    private val autoChooser = SendableChooser<Command>()

    private val namedCommandsList = mapOf(
        "Rev Hub" to HoodedShooter.rev(),
        "Rev Stop" to HoodedShooter.stop(),

        "Intake Down" to Grintake.intake(),
        "Intake Up" to Grintake.stow(),

        "Shoot" to HoodedShooter.shoot(),
        "Stop Shoot" to HoodedShooter.stop()
    )

    init {
        NamedCommands.registerCommands(namedCommandsList)

        AutoBuilder.configure(
            Drivetrain::getPose,
            Drivetrain::setPose,
            Drivetrain::getChassisSpeeds,
            Drivetrain::driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(
                    SwerveConstants.AUTO_TRANSLATION_P,
                    SwerveConstants.AUTO_TRANSLATION_I,
                    SwerveConstants.AUTO_TRANSLATION_D
                ),
                PIDConstants(
                    SwerveConstants.AUTO_HEADING_P,
                    SwerveConstants.AUTO_HEADING_I,
                    SwerveConstants.AUTO_HEADING_D
                )
            ),
            RobotConfig(
                Pounds.of(115.0),
                KilogramSquareMeters.of(1.0),
                ModuleConfig(
                    SwerveConstants.wheelRadius,
                    SwerveConstants.maxSpeed,
                    SwerveConstants.WHEEL_COF,
                    DCMotor.getNEO(1),
                    SwerveConstants.driveGearing.mechanismToRotorRatio,
                    Amps.of(40.0),
                    1
                )
            ),
            { DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red }
        )

        autoChooser.addOption("", AutoBuilder.buildAuto(""))
        autoChooser.setDefaultOption("None", Commands.none())
    }

    fun getAutonomousCommand():Command{
        return autoChooser.selected
    }
}