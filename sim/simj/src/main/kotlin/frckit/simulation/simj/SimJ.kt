package frckit.simulation.simj

import javafx.application.Application
import javafx.scene.Scene
import javafx.scene.control.Button
import javafx.scene.layout.Pane
import javafx.stage.Stage

class SimJ: Application() {
    override fun start(primaryStage: Stage) {
        val pane = Pane(Button("Test"))
        val scene = Scene(pane)
        primaryStage.scene = scene
        primaryStage.show()
    }
}

fun main() {
    Application.launch(SimJ::class.java)
}