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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import org.sert2521.rebuilt2026.commands.HoodedShooterCommands
import org.sert2521.rebuilt2026.subsystems.Intake
import org.sert2521.rebuilt2026.subsystems.Indexer
import org.sert2521.rebuilt2026.subsystems.Wrist
import org.sert2521.rebuilt2026.subsystems.drivetrain.Drivetrain
import org.sert2521.rebuilt2026.subsystems.drivetrain.SwerveConstants
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Flywheel
import org.sert2521.rebuilt2026.subsystems.hooded_shooter.Hood
import kotlin.jvm.optionals.getOrElse

object Autos {
    private val autoChooser = SendableChooser<Command>()

    private val namedCommandsList = mapOf(
        "Rev Pass" to ScheduleCommand(HoodedShooterCommands.revAndTrackPass()).asProxy(),
        "Rev Stop" to HoodedShooterCommands.stop(),

        "Intake Down" to ScheduleCommand(Intake.fullSpeedIntake().alongWith(Wrist.down()).alongWith(Indexer.manualIndex())).asProxy(),
        "Intake Up" to ScheduleCommand(Wrist.up().alongWith(Intake.stop()).alongWith(Indexer.index())).asProxy(),

        "Depot Inter" to ScheduleCommand(Wrist.toDepotInter().alongWith(Intake.fullSpeedIntake()).alongWith(Indexer.manualIndex())).asProxy(),
        "Depot" to ScheduleCommand(Wrist.toDepot().alongWith(Intake.fullSpeedIntake()).alongWith(Indexer.manualIndex())).asProxy(),

        "Rev" to ScheduleCommand(HoodedShooterCommands.revAndTrackHub()).asProxy(),
        "Shoot" to ScheduleCommand(Indexer.pulse().alongWith(runOnce(Flywheel::startTimer))).asProxy(),
        "Stop Shoot" to ScheduleCommand(Indexer.index().alongWith(runOnce(Flywheel::stopTimer))).asProxy(),
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
                Pounds.of(130.0),
                KilogramSquareMeters.of(4.8858286294),
                ModuleConfig(
                    SwerveConstants.wheelRadius,
                    SwerveConstants.maxSpeed,
                    SwerveConstants.WHEEL_COF,
                    DCMotor.getNEO(1),
                    SwerveConstants.driveGearing.mechanismToRotorRatio,
                    Amps.of(40.0),
                    1
                ),
                *SwerveConstants.moduleTranslations
            ),
            { DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue } == DriverStation.Alliance.Red },
            Drivetrain
        )

        autoChooser.addOption("L_N", AutoBuilder.buildAuto("CL_N"))
        autoChooser.addOption("L_N_D", AutoBuilder.buildAuto("CL_N_D"))
        autoChooser.addOption("L_D", AutoBuilder.buildAuto("CL_D"))
        autoChooser.addOption("R_N", AutoBuilder.buildAuto("CR_N"))
        autoChooser.addOption("R_N_N", AutoBuilder.buildAuto("CR_N_N"))
        autoChooser.addOption("L_N_N", AutoBuilder.buildAuto("CL_N_N"))
        autoChooser.addOption("C_", AutoBuilder.buildAuto("C_"))
        autoChooser.setDefaultOption("None", Commands.none())

        SmartDashboard.putData("Autos", autoChooser)
    }

    fun getAutonomousCommand():Command{
        return autoChooser.selected
    }
}