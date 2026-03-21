package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import org.sert2521.rebuilt2026.util.AllianceShiftUtil
import org.sert2521.rebuilt2026.util.HSGoal
import org.sert2521.rebuilt2026.util.HSMapDatapoint
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity
import yams.telemetry.MechanismTelemetry

object TelemetryConstants {
    val DRIVETRAIN_ANGLE_TELEMETRY = TelemetryVerbosity.LOW
    val DRIVETRAIN_DRIVE_TELEMETRY = TelemetryVerbosity.LOW

    val GRINTAKE_TELEMETRY = TelemetryVerbosity.LOW
    val INDEXER_TELEMETRY = TelemetryVerbosity.MID

    val HOODED_SHOOTER_TELEMETRY = TelemetryVerbosity.MID
}

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 21
    const val GRINTAKE_WRIST_MOTOR_ID = 22

    const val INDEXER_MOTOR_ID = 31
    const val KICKER_MOTOR_ID = 32

    const val FLYWHEEL_LEFT_ID = 41
    const val FLYWHEEL_RIGHT_ID = 42
    const val SHOOTER_ROLLER_MOTOR_ID = 43
    const val HOOD_MOTOR_ID = 44

    const val INDEXER_BEAM_BREAK_ID = 1
}

object GrintakeConstants {
    const val WRIST_P = 6.4
    const val WRIST_D = 0.0

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            24.0/12.0,
            36.0/24.0
        )
    )
    val wristGearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0,
            4.0,
            54.0/24.0,
            56.0/24.0
        )
    )

    val stowPosition = Rotations.of(0.07)
    val depotPosition = Rotations.of(0.393422)
    val depotInter = depotPosition - Degrees.of(10.0)
    val intakePosition = Rotations.of(0.42) - Degrees.of(10.0)

    val intakeVoltageAuto = Volts.of(12.0)
    val intakeVoltage = Volts.of(6.0)
    val reverseVoltage = Volts.of(0.0)

    const val REZERO_SPEED = -0.2
    val reZeroThreshold = Amps.of(20.0)
}

object IndexerConstants {
    val indexerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            4.0,
            28.0/24.0
        )
    )
    val kickerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            3.0
        )
    )

    const val MAIN_DEFAULT = 0.0
    const val KICKER_DEFAULT = 0.0

    const val MAIN_INDEXING = 0.4
    const val KICKER_INDEXING = -0.4

    const val MAIN_KICKING = 0.7
    const val KICKER_KICKING = 0.9
    const val KICK_TIME = 0.0

    const val MAIN_REVERSE = -1.0
    const val KICKER_REVERSE = -1.0

    const val PULSE_SHOOT_TIME = 100.0
    const val PULSE_DELAY_TIME = 0.0
}

object ShooterConstants {
    const val F_P = 0.02
    const val F_D = 0.001
    const val F_S = 0.0
    const val F_V = 0.152
    const val F_A = 0.0

    const val R_P = 0.0
    const val R_D = 0.0
    const val R_S = 0.0
    const val R_V = 0.0

    const val H_P = 0.0
    const val H_D = 0.0

    val hoodOffset = Rotations.of(0.0)

    val shooterGearing = MechanismGearing(
        GearBox.fromReductionStages(
            24.0 / 20.0
        )
    )

    val rollerGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0
        )
    )

    val hoodGearing = MechanismGearing(
        GearBox.fromReductionStages(
            1.0 // TODO: Change
        )
    )

    val primaryShootFlywheel = RPM.of(3100.0)
    val secondaryShootFlywheel = RPM.of(2300.0)
    val shootRollerDutyCycle = Volts.of(6.0)

    val primaryPassFlywheel = RPM.of(2000.0)
    val secondaryPassFlywheel = RPM.of(1000.0)
    val passRollerDutyCycle = Volts.of(12.0)
}

object OtherConstsants {
    val fieldWidth = Inches.of(317.6875)
    val fieldLength = Inches.of(651.25)

    val blueHubTranslation = Translation2d(4.620755195617676,4.036807537078857)
    val redHubTranslation = Translation2d(11.91766357421875, 4.037060737609863)

    val blueBumps = arrayOf(Translation2d(4.0, 2.0), Translation2d(4.0, 2.0).flipWidth())
    val redBumps = arrayOf(blueBumps[0].flipAlliance(), blueBumps[1].flipAlliance())

    val currentHub = if (AllianceShiftUtil.allianceIsBlue()) { blueHubTranslation } else { redHubTranslation }
    val currentBumps = if (AllianceShiftUtil.allianceIsBlue()) { blueBumps } else { redBumps }

    val dataHub = arrayOf(
        HSMapDatapoint(
            0.0,
            2500.0,
            2300.0,
            2000.0,
            0.0
        )
    )

    val dataPass = arrayOf(
        HSMapDatapoint(
            0.0,
            1000.0,
            1000.0,
            2000.0,
            0.0
        )
    )
}

fun Translation2d.flipWidth(): Translation2d{
    return Translation2d(this.x, OtherConstsants.fieldWidth.`in`(Meters) - this.y)
}

fun Translation2d.flipAlliance(): Translation2d{
    return Translation2d(OtherConstsants.fieldLength.`in`(Meters)-this.x, this.y)
}