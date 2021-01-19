package frckit.simulation.simj

import javafx.application.Application
import javafx.application.ConditionalFeature
import javafx.application.Platform
import javafx.scene.Scene
import javafx.scene.control.Button
import javafx.scene.layout.Pane
import javafx.scene.shape.Box
import javafx.scene.transform.Rotate
import javafx.stage.Stage

class SimJ: Application() {
    override fun start(primaryStage: Stage) {
        println(Platform.isSupported(ConditionalFeature.SCENE3D))

        val box = Box()
        val test = Rotate()
    }
}

fun main() {
    Application.launch(SimJ::class.java)
}