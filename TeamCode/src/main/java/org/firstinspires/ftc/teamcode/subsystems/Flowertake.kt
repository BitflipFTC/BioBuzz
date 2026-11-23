package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Flowertake(hardwareMap: HardwareMap, telemetry: Telemetry) :
    Subsystem(hardwareMap, telemetry) {

    enum class Mode(val position: Double) {
        DOWN(0.0),
        UP(0.6)
    }

    private val servo = ServoEx("flower")

    init {
        up()
    }

    var mode = Mode.UP
        set(newMode) {
            field = newMode
            servo.position = newMode.position + offset
        }

    var offset: Double = 0.0
    var lastOffset = offset

    override fun loop() {
        if (offset != lastOffset) {
            lastOffset = offset
            servo.position = mode.position + offset
        }

        telemetry.addData("Flowertake Mode", mode.name)
    }

    fun up() {
        mode = Mode.UP
    }

    fun down() {
        mode = Mode.DOWN
    }

    fun toggle() {
        mode = if (mode == Mode.UP) {
            Mode.DOWN
        } else {
            Mode.UP
        }
    }
}