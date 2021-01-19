package frckit.simulation.simj

import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.ode4j.math.DQuaternion
import org.ode4j.math.DVector3
import org.ode4j.ode.*

fun main() {
    OdeHelper.initODE2(0)

    val world = OdeHelper.createWorld()
    val space = OdeHelper.createSimpleSpace()
    val contactGroup = OdeHelper.createJointGroup()

    val plane = OdeHelper.createPlane(space, 0.0, 0.0, 1.0, 0.0)
    world.setGravity(0.0, 0.0, -9.81)

    world.contactMaxCorrectingVel = 0.9
    world.contactSurfaceLayer = .001

    val body = OdeHelper.createBody(world)
    body.setPosition(0.0, 0.0, 5.0)

    val q = DQuaternion()
    OdeMath.dQFromAxisAndAngle(q, 1.0, 0.0, 0.0, Math.toRadians(30.0))
    body.quaternion = q

    val sides = DVector3(2.0, 2.0, 2.0)
    val mass = OdeHelper.createMass()
    mass.setBox(0.5, sides)
    mass.adjust(100.0)

    body.mass = mass

    val box = OdeHelper.createBox(space, sides)
    box.body = body


    val nearCallback = DGeom.DNearCallback { data, o1, o2 ->
        val b1 = o1.body
        val b2 = o2.body

        val contacts = DContactBuffer(20)
        val n = OdeHelper.collide(o1, o2, 20, contacts.geomBuffer)
        for (i in 0 until n) {
            val contact = contacts[i]
            contact.surface.mode = OdeConstants.dContactApprox1
            contact.surface.mu = 1.0

            val joint = OdeHelper.createContactJoint(world, contactGroup, contact)
            joint.attach(b1, b2)

        }
    }

    val time = DoubleArray(20000)
    val height = DoubleArray(20000)

    val start = System.nanoTime()
    for (i in 0 until 20000) {
        time[i] = i * .0001
        height[i] = box.position.get2()

        space.collide(null, nearCallback)

        world.step(.0001)

        contactGroup.empty()
    }
    val end = System.nanoTime()

    println((end - start) * 1e-6)

    SwingWrapper(QuickChart.getChart("Falling", "t", "z", "z(t)", time, height)).displayChart()
}