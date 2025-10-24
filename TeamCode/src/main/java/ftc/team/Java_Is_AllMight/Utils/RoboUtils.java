package ftc.team.Java_Is_AllMight.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoboUtils {

    public <T> T getHardware(HardwareMap hardwareMap, Class<T> tipoDeHardware, String nome){
        return hardwareMap.get(tipoDeHardware, nome);

    }
}
