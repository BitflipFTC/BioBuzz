package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    enum class Mode(val power: Double) {
        OFF(0.0),
        SHOOTING(0.7),
        ON(1.0),
    }

    private val motorL = MotorEx("intakeL").zeroed().float().reverse()
    private val motorR = MotorEx("intakeR").zeroed().float()

    var mode = Mode.OFF
        set(newMode) {
            field = newMode
            motorL.power = newMode.power
            motorR.power = newMode.power
        }

    override fun loop() {
        telemetry.addData("Intake Mode", mode.name)
    }

    fun on() {
        mode = Mode.ON
    }

    fun off() {
        mode = Mode.OFF
    }

    fun toggle() {
        mode = if (mode == Mode.ON) {
            Mode.OFF
        } else {
            Mode.ON
        }
    }
}