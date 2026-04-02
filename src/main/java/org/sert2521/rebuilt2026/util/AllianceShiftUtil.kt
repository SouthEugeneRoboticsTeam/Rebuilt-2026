package org.sert2521.rebuilt2026.util

import dev.doglog.DogLog
import edu.wpi.first.wpilibj.DriverStation
import kotlin.jvm.optionals.getOrElse
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.sert2521.rebuilt2026.Input
import kotlin.math.floor

object AllianceShiftUtil {
    enum class Shift{
        AUTO,
        TRANSITION,
        SHIFT_ONE,
        SHIFT_TWO,
        SHIFT_THREE,
        SHIFT_FOUR,
        ENDGAME
    }

    var shiftTime = 0.0
    var blipStrength = 0.0
    val rumbleBlip = Input.rumbleBlip { blipStrength }


    private fun getAllianceShift(time:Double): Shift{
        return when {
            time > 140.0 -> Shift.AUTO
            time in 130.0..140.0 -> Shift.TRANSITION
            time in 105.0..130.0 -> Shift.SHIFT_ONE
            time in 80.0..105.0 -> Shift.SHIFT_TWO
            time in 55.0..80.0 -> Shift.SHIFT_THREE
            time in 30.0..55.0 -> Shift.SHIFT_FOUR
            else -> Shift.ENDGAME
        }
    }

    private fun getAllianceShiftText(shift:Shift):String{
        return when (shift) {
            Shift.AUTO -> "Auto"
            Shift.TRANSITION -> "Transition"
            Shift.SHIFT_ONE -> "Shift One"
            Shift.SHIFT_TWO -> "Shift Two"
            Shift.SHIFT_THREE -> "Shift Three"
            Shift.SHIFT_FOUR -> "Shift Four"
            Shift.ENDGAME -> "Endgame"
        }
    }

    private fun getAllianceShiftTime(shift: Shift, time: Double): Double {
        return when (shift) {
            Shift.AUTO -> time - 140.0
            Shift.TRANSITION -> time - 130.0
            Shift.SHIFT_ONE -> time - 105.0
            Shift.SHIFT_TWO -> time - 80.0
            Shift.SHIFT_THREE -> time - 55.0
            Shift.SHIFT_FOUR -> time - 30.0
            Shift.ENDGAME -> time
        }
    }

    private fun getAllianceShiftColor(shift: Shift):DriverStation.Alliance{
        val alliance = DriverStation.getAlliance().getOrElse { DriverStation.Alliance.Blue }

        val opponent = when (alliance){
            DriverStation.Alliance.Blue -> DriverStation.Alliance.Red
            DriverStation.Alliance.Red -> DriverStation.Alliance.Blue
            else -> DriverStation.Alliance.Red
        }

        val autoWon = if (DriverStation.getGameSpecificMessage() == "R"){
            DriverStation.Alliance.Red
        } else {
            DriverStation.Alliance.Blue
        }
        return when (shift) {
            Shift.AUTO -> alliance
            Shift.TRANSITION -> alliance
            Shift.SHIFT_ONE -> if (autoWon == alliance) { opponent } else { alliance }
            Shift.SHIFT_TWO -> if (autoWon == alliance) { alliance } else { opponent }
            Shift.SHIFT_THREE -> if (autoWon == alliance) { opponent } else { alliance }
            Shift.SHIFT_FOUR -> if (autoWon == alliance) { alliance } else { opponent }
            Shift.ENDGAME -> alliance
        }
    }

    fun update(){
        val time = DriverStation.getMatchTime()
        val shift = getAllianceShift(time)
        shiftTime = floor(getAllianceShiftTime(shift, time) * 10.0) / 10.0
        val dashboardTime = (shiftTime).toString()

        if (shiftTime <= 5.0 && shiftTime >= 0.0 && DriverStation.isEnabled()) {
            Input.setRumble(0.2)
        } else {
            Input.setRumble(0.0)
        }

        DogLog.log("Alliance Shift/Timer",
            if ("." in dashboardTime){
                dashboardTime
            } else {
                "$dashboardTime.0"
            }
        )
        DogLog.log("Alliance Shift/Shift", getAllianceShiftText(shift))
        DogLog.log("Alliance Shift/Current Scorer",
            when (getAllianceShiftColor(shift)) {
                DriverStation.Alliance.Blue -> Color.kBlue.toHexString()
                DriverStation.Alliance.Red -> Color.kRed.toHexString()
            }
        )
    }
}