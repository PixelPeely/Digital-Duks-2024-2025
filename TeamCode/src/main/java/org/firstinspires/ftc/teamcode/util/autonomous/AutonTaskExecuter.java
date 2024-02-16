package org.firstinspires.ftc.teamcode.util.autonomous;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardInterface;
import org.firstinspires.ftc.teamcode.util.DukConstants;
import org.firstinspires.ftc.teamcode.util.DukUtilities;
import org.firstinspires.ftc.teamcode.util.PersistentData;
import org.firstinspires.ftc.teamcode.util.TimeManager;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;

public class AutonTaskExecuter {
    public final Queue<AutonTask> tasks = new ArrayDeque<>();
    private final List<AutonTask> asynchronousTasks = new ArrayList<>();

    /**
     * Execute next task in the queue
     * @return True if there are no more tasks left
     */
    public boolean tick() {
        if (tasks.size() == 0) return true;
        AutonTask currentTask = tasks.peek();

        if (currentTask.runAsynchronous()) {
            asynchronousTasks.add(currentTask);
            tasks.remove();
            return false;
        }
        executeTask(currentTask);
        asynchronousTasks.forEach(this::executeTask);
        return false;
    }

    private void executeTask(AutonTask task) {
        if (task.shouldTerminate()) {
            task.onTerminate();
            tasks.remove(task);
            asynchronousTasks.remove(task);
            if (tasks.size() > 0)
                tasks.peek().initialize();
        }
        else
            task.execute();
    }

    public void renderPoints() {
//        if (tasks.size() < 2) return;
//        Iterator<AutonTask> futurePoints = tasks.iterator();
//        futurePoints.remove();
//        futurePoints.forEachRemaining(point -> DashboardInterface.renderRobot(
//                DukConstants.DEBUG.ROBOT_POINT_STROKE,
//                point,
//                point.y,
//                point.h));
    }

    public void terminate() {
        PersistentData.available = true;
        PersistentData.positionX = DukHardwareMap.instance.odometerWheels.positionX;
        PersistentData.positionY = DukHardwareMap.instance.odometerWheels.positionY;
        PersistentData.heading = DukUtilities.differenceConstrained(DukHardwareMap.instance.odometerWheels.headingOffset, DukHardwareMap.instance.odometerWheels.getHeading(true));
    }
}
