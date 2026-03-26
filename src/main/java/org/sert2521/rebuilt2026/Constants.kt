package org.sert2521.rebuilt2026

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import org.sert2521.rebuilt2026.util.AllianceShiftUtil
import org.sert2521.rebuilt2026.util.HSMapDatapoint
import yams.gearing.GearBox
import yams.gearing.MechanismGearing
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity

object TelemetryConstants {
    val DRIVETRAIN_ANGLE_TELEMETRY = TelemetryVerbosity.LOW
    val DRIVETRAIN_DRIVE_TELEMETRY = TelemetryVerbosity.LOW

    val GRINTAKE_TELEMETRY = TelemetryVerbosity.LOW
    val INDEXER_TELEMETRY = TelemetryVerbosity.LOW

    val HOODED_SHOOTER_TELEMETRY = TelemetryVerbosity.HIGH
}

object ElectronicIDs {
    const val GRINTAKE_ROLLER_MOTOR_ID = 21
    const val GRINTAKE_WRIST_MOTOR_ID = 22

    const val INDEXER_MOTOR_ID = 31
    const val KICKER_MOTOR_ID = 32

    const val FLYWHEEL_LEFT_ID = 41
    const val FLYWHEEL_RIGHT_ID = 42
    const val HOOD_MOTOR_ID = 43
    const val HOOD_ABSOLUTE_ENCODER_ID = 44

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

    val stowPosition = Rotations.of(0.07) - Degrees.of(8.0)
    val depotPosition = Rotations.of(0.393422)
    val depotInter = depotPosition - Degrees.of(10.0)
    val intakePosition = Rotations.of(0.42) - Degrees.of(10.0)

    val intakeVoltageAuto = Volts.of(12.0)
    val intakeVoltage = Volts.of(8.0)
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

    const val MAIN_KICKING = 0.5
    const val KICKER_KICKING = 0.7
    const val KICK_TIME = 0.0

    const val MAIN_REVERSE = -1.0
    const val KICKER_REVERSE = -1.0

    const val PULSE_SHOOT_TIME = 100.0
    const val PULSE_DELAY_TIME = 0.0
}

object ShooterConstants {
    const val F_P = 0.03
    const val F_D = 0.008
    const val F_S = 0.0
    const val F_V = 0.152
    const val F_A = 0.0

    const val H_S = 0.2
    const val H_P = 10.0
    const val H_D = 1.0


    val hoodOffset = Rotations.of(0.141)
    const val HOOD_ABSOLUTE_ENCODER_GEARING = 150.0/14.0

    val shooterGearing = MechanismGearing(
        GearBox.fromReductionStages(
            24.0 / 20.0
        )
    )

    val hoodGearing = MechanismGearing(
        GearBox.fromReductionStages(
            150.0/14.0,
            10.0
        )
    )

    val hoodMax = Rotations.of(0.075)
    val hoodMin = Rotations.of(0.0)
    val hoodSoftMax = hoodMax - Degrees.of(2.0)
    val hoodSoftMin = hoodMin + Degrees.of(1.0)

    val shotTime = Seconds.of(999.9)
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
        // Remember to put in order of distance
        HSMapDatapoint(
            1.5,
            2400.0,
            0.0
        ),
        HSMapDatapoint(
            1.8,
            2400.0,
            0.01
        ),
        HSMapDatapoint(
            2.28,
            2400.0,
            0.02
        ),
        HSMapDatapoint(
            2.916,
            2230.0,
            0.04072
        ),
        HSMapDatapoint(
            3.4,
            2410.0,
            0.044
        ),
        HSMapDatapoint(
            4.3,
            2610.0,
            0.052
        ),
        HSMapDatapoint(
            5.0,
            2860.0,
            0.06
        )
    )

    val dataPass = arrayOf(
        HSMapDatapoint(
            0.0,
            2500.0,
            0.075
        ),
        HSMapDatapoint(
            5.0,
            3000.0,
            0.075
        ),
        HSMapDatapoint(
            8.0,
            4000.0,
            0.075
        ),
        HSMapDatapoint(
            10.0,
            4500.0,
            0.075
        )

    )

    var flywheelLiveSetpoint = RPM.of(2600.0)

    val distanceSpeedAdjustment = Seconds.of(1.4)
}

fun Translation2d.flipWidth(): Translation2d{
    return Translation2d(this.x, OtherConstsants.fieldWidth.`in`(Meters) - this.y)
}

fun Translation2d.flipAlliance(): Translation2d{
    return Translation2d(OtherConstsants.fieldLength.`in`(Meters)-this.x, this.y)
}