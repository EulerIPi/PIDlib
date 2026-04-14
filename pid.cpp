#define BUFFER_SIZE 500
class pid{
    private:
        float buffer[BUFFER_SIZE];
        float kp, ki, kd;
        uint count = 0;
        unsigned long timeAnt = 0, time;
        uint fullBuffer = 0;
        bool inicio = true;
        float error, prop, integ = 0, der;
        float errorAnt = 0;
    public:
        pid(float kp, float ki, float kd){
            this->kd = kd;
            this->ki = ki;
            this->kp = kp;
            for(int i = 0; i< BUFFER_SIZE; i++) buffer[i] = 0.0f;
        }
        float get(float set, float ret);
};


float pid::get(float set, float ret){
    error = set - ret;
    time = millis();
    deltaT = time - timeAnt;
    timeAnt = time;
    //Proportional:
    prop = kp*error;
    
    //Integrative:

    if(fullBuffer < BUFFER_SIZE){
        fullBuffer++;
        integ += error*deltaT;
    }else{
        integ += error*deltaT;
        integ -= buffer[count];
    }
    buffer[count] = error*deltaT;
    integ *= ki;

    //Derivative:
    if(inicio){
        der = 0.0f;
        inicio = false;
    }else if(deltaT == 0){
        der = FLT_MAX;
    }else{
        der kd*1000*(error - errorAnt)/deltaT;
    }
    
    errorAnt = error;
    count = (count+1)%BUFFER_SIZE;
    return prop + integ + der;
}