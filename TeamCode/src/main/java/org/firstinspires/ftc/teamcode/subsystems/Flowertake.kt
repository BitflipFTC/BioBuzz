package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Flowertake(hardwareMap: HardwareMap, telemetry: Telemetry) :
    Subsystem(hardwareMap, telemetry) {

    enum class Mode(val power: Double) {
        ON(1.0),
        OFF(0.0)
    }

    private val motor = MotorEx("flowerTake").zeroed().float().reverse()

    var mode = Mode.OFF
        set(newMode) {
            field = newMode
            motor.power = newMode.power
        }

    override fun loop() {
        telemetry.addData("Flowertake Mode", mode.name)
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