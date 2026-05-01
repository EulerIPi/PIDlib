PID::PID(float kp, float ki, float kd, float constReset = 5000.0f, float MAXall = 9999.0f, float MAXp = 9999.0f, float MAXi = 9999.0f, float MAXd = 9999.0f){
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
    this->constReset = constReset;
    this-> MAXall = MAXall;
    this-> MAXp = MAXp;
    this-> MAXi = MAXi;
    this-> MAXd = MAXd;
    for(int i = 0; i< BUFFER_SIZE; i++) buffer[i] = 0.0f;
    auxInteg = 0.0f;
    fullBuffer = 0;
    count = 0;
    integ = 0.0f;
}

float PID::limitter(float val, float lim){
    if(val > lim){
        return lim;
    }else if(val < -lim){
        return -lim;
    }
    return val;
}

float PID::get(float set, float ret){
    if(set - setAnt > constReset || setAnt - set > constReset){
        for(int i = 0; i< BUFFER_SIZE; i++) buffer[i] = 0.0f;
        start = true;
        errorAnt = 0.0f;
        auxInteg = 0;
        fullBuffer = 0;
        count = 0;
    }
    setAnt = set;
    
    error = set - ret;
    time = millis();
    deltaT = time - timeAnt;
    timeAnt = time;

    //Proportional:
    prop = limitter(kp*error, MAXp);

    //Integrative:
    if(fullBuffer < BUFFER_SIZE){
        fullBuffer++;
        auxInteg += (error+errorAnt)*deltaT*0.2f;
    }else{
        auxInteg += (error+errorAnt)*deltaT*0.2f;
        auxInteg -= buffer[count];
    }
    buffer[count] = error*deltaT;
    integ = limitter(auxInteg*ki, MAXi);
    
    //Derivative:
    if(start){
        der = 0.0f;
        start = false;
    }else if(deltaT != 0){
        der = limitter(kd*1000*(error - errorAnt)/deltaT, MAXd);
    }
    
    errorAnt = error;
    count = (count+1)%BUFFER_SIZE;
    return limitter(prop + integ + der, MAXall);
}
