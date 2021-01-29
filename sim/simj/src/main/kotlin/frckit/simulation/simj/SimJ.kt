package frckit.simulation.simj

import javafx.animation.AnimationTimer
import javafx.application.Application
import javafx.geometry.Point3D
import javafx.scene.Group
import javafx.scene.Node
import javafx.scene.PerspectiveCamera
import javafx.scene.Scene
import javafx.scene.paint.Color
import javafx.scene.paint.PhongMaterial
import javafx.scene.shape.Box
import javafx.scene.shape.CullFace
import javafx.scene.shape.Cylinder
import javafx.scene.shape.Sphere
import javafx.scene.transform.Rotate
import javafx.scene.transform.Transform
import javafx.scene.transform.Translate

import javafx.stage.Stage
import net.java.games.input.ControllerEnvironment
import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.ode4j.math.DQuaternion
import org.ode4j.math.DQuaternionC
import org.ode4j.math.DVector3
import org.ode4j.math.DVector3C
import org.ode4j.ode.OdeHelper
import org.ode4j.ode.OdeMath
import java.util.concurrent.Executors
import java.util.concurrent.ThreadFactory
import java.util.concurrent.TimeUnit
import kotlin.math.*

class SimJ: Application() {
    private val eps = Math.ulp(1.0)
    private val axisRotate = Rotate(-90.0, Rotate.X_AXIS)

    private val boxRotate = Rotate()
    private val boxTranslate = Translate()

    fun placeElement(element: Node, translation: DVector3C, rotation: DQuaternionC) {
    }

    fun updateCube(box: Box, simulation: Simulation) {
        //val state = Simulation.State(vecTest, qTest)

        val state = simulation.currentState

        val t = state.position

        //boxTranslate.x = t.get0()
        //boxTranslate.y = t.get1()
        //boxTranslate.z = t.get2()

        box.translateX = t.get0()
        box.translateY = t.get1()
        box.translateZ = t.get2()

        val q = state.rotation
        val angle = 2 * acos(q.get0())
        val s = sqrt(1.0 - q.get0() * q.get0())
        val x: Double
        val y: Double
        val z: Double
        if (s < eps) {
            x = q.get1()
            y = q.get2()
            z = q.get3()
        } else {
            x = q.get1() / s
            y = q.get2() / s
            z = q.get3() / s
        }
        box.rotationAxis = Point3D(x, y, z)
        box.rotate = Math.toDegrees(angle)

        //boxRotate.axis = Point3D(x, y, z)
        //boxRotate.angle = Math.toDegrees(angle)
    }

    override fun start(primaryStage: Stage) {
        OdeHelper.initODE2(0)
        val simulation = Simulation()
        val executor = Executors.newSingleThreadScheduledExecutor { Thread(it).apply { isDaemon = true } }

        val phongRed = PhongMaterial(Color.RED)
        val phongGreen = PhongMaterial(Color.GREEN)
        val phongBlue = PhongMaterial(Color.BLUE)
        val phongMagenta = PhongMaterial(Color.MAGENTA)

        val originSphere = Sphere(0.05)
        originSphere.material = phongMagenta

        val floorBox = Box(100.0, 100.0, 0.001)
        floorBox.material = PhongMaterial(Color.ORANGE)

        val xCyl = Cylinder(0.01, 1.0)
        xCyl.rotationAxis = Rotate.Z_AXIS
        xCyl.rotate = 90.0
        xCyl.translateX = 0.5
        xCyl.material = phongRed

        val yCyl = Cylinder(0.01, 1.0)
        yCyl.translateY = 0.5
        yCyl.material = phongGreen

        val zCyl = Cylinder(0.01, 1.0)
        zCyl.rotationAxis = Rotate.X_AXIS
        zCyl.rotate = 90.0
        zCyl.translateZ = 0.5
        zCyl.material = phongBlue

        val box = Box(1.0, 1.0, 1.0)
        box.cullFace = CullFace.NONE
        box.material = phongRed

        //box.transforms.addAll(boxTranslate, boxRotate)


        val camera = PerspectiveCamera(true)


        val root = Group(floorBox, originSphere, xCyl, yCyl, zCyl, box)

        val scene = Scene(root, 500.0, 300.0, true)
        scene.camera = camera
        primaryStage.scene = scene
        primaryStage.title = "Test"

        primaryStage.show()

        val environment = ControllerEnvironment.getDefaultEnvironment()
        val controllers = environment.controllers
        val spaceMouse = controllers.first { it.name.startsWith("SpaceMouse") }
        val components = spaceMouse.components

        val transDiv = 500.0
        val rotDiv = 60.0

        val xAxis = components.first { it.name == "X Axis" }
        val yAxis = components.first { it.name == "Y Axis" }
        val zAxis = components.first { it.name == "Z Axis" }
        val xRotation = components.first { it.name == "X Rotation" }
        val zRotation = components.first { it.name == "Z Rotation" }

        val cRotateX = Rotate(0.0, Rotate.X_AXIS)
        val cRotateY = Rotate(0.0, Rotate.Y_AXIS)
        val cRotateZ = Rotate(0.0, Rotate.Z_AXIS)
        val cTranslate = Translate(0.0, 0.0, 0.0)
        camera.transforms.addAll(axisRotate, cRotateY, cRotateX, cRotateZ, cTranslate)

        val animationTimer = object : AnimationTimer() {
            override fun handle(now: Long) {
                updateCube(box, simulation)

                if (spaceMouse.poll()) {
                    cTranslate.x  = cTranslate.x + xAxis.pollData / transDiv
                    cTranslate.y = cTranslate.y + zAxis.pollData / transDiv
                    cTranslate.z = cTranslate.z - yAxis.pollData / transDiv

                    cRotateX.angle = cRotateX.angle + xRotation.pollData / rotDiv
                    cRotateY.angle = cRotateY.angle + zRotation.pollData / rotDiv
                }
            }

        }

        animationTimer.start()

        executor.scheduleAtFixedRate(simulation::update, 5000L, 10L, TimeUnit.MILLISECONDS)
    }
}

fun main() {
    OdeHelper.initODE2(0)
    /*
    val sim = Simulation()

    val times = DoubleArray(1000)
    val heights = DoubleArray(1000)
    for (i in 0 until 1000) {
        times[i] = i * .01
        heights[i] = sim.currentState.position.get2()
        sim.update()
    }

    SwingWrapper(QuickChart.getChart("Test", "t", "z", "z(t)", times, heights)).displayChart()

     */
    Application.launch(SimJ::class.java)

}