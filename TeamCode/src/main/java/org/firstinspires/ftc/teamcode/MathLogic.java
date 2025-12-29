package org.firstinspires.ftc.teamcode;

public class MathLogic {

    /**
     * zet een double tussen een bepaalde range
     * @param _N de input
     * @param _LowerBound de laagste waarde, (negatief)
     * @param _UpperBound de hoogste waarde
     * @return returns de waarde
     */
    public static double Clamp(double _N, double _LowerBound, double _UpperBound){

        if(_N < _LowerBound){
            return _LowerBound;
        }
        if(_N > _UpperBound){
            return _UpperBound;
        }

        return _N;
    }
}
