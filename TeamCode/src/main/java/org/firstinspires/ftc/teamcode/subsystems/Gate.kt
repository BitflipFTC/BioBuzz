package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.ServoEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Gate(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    enum class State(val position: Double) {
        OPEN(0.375),
        CLOSED(0.5)
    }

    private val servo: ServoEx = ServoEx("gate")
    var state = State.CLOSED
        set(newState) {
            field = newState
            servo.position = newState.position
        }

    init {
        servo.scaleRange(State.OPEN.position, State.CLOSED.position)
        close()
    }

    override fun loop() {
        telemetry.run {
            addData("Gate status", state.name)
        }
    }

    fun close() {
        state = State.CLOSED
    }

    fun open() {
        state = State.OPEN
    }

    fun toggle() {
        if (state == State.CLOSED) {
            open()
        } else {
            close()
        }
    }
}