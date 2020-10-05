import frckit.simulation.protocol.RobotCycle;

public class test {
    public static void main(String[] args) {
        RobotCycle.RobotCycleMessage message = RobotCycle.RobotCycleMessage.newBuilder()
                .addPidConfigCommands(RobotCycle.PIDConfigCommand.newBuilder()
                        .setKI(5.0)
                        .build()
                ).build();

        RobotCycle.PIDConfigCommand command = message.getPidConfigCommands(0);

        switch(command.getSettingCase()) {
            case KP:

        }
        message.getPidConfigCommands(0).getSettingCase();



    }
}
