package pandaPathing.subsytem;


import com.arcrobotics.ftclib.command.SubsystemBase;

//SubsystemBase
public abstract class WSubsystem extends SubsystemBase {
    abstract public void read();
    abstract public void loop();
    abstract public void write();

}
