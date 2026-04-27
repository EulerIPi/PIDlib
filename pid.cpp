PID(float kp, float ki, float kd, float constReset = 5000.0f, float MAXall = 9999.0f, float MAXp = 9999.0f, float MAXi = 9999.0f, float MAXd = 9999.0f){
    this->kd = kd;
    this->ki = ki;
    this->kp = kp;
    this->constReset = constReset;
    this-> MAXall = MAXall;
    this-> MAXp = MAXp;
    this-> MAXi = MAXi;
    this-> MAXd = MAXd;
    for(int i = 0; i< BUFFER_SIZE; i++) buffer[i] = 0.0f;
}

float PID::limitter(float val, float lim){
    return (val > lim || -val < -lim)? lim : val;
}

float PID::get(float set, float ret){
    error = set - ret;
    time = millis();
    deltaT = time - timeAnt;
    timeAnt = time;

    if(error > constReset){
        for(int i = 0; i< BUFFER_SIZE; i++) buffer[i] = 0.0f;
        start = true;
        errorAnt = 0.0f;
    }

    //Proportional:
    prop = constrainPID(kp*error, MAXp);
    
    //Integrative:

    if(fullBuffer < BUFFER_SIZE){
        fullBuffer++;
        integ += error*deltaT;
    }else{
        integ += error*deltaT;
        integ -= buffer[count];
    }
    buffer[count] = error*deltaT;
    integ = constrainPID(integ*ki, MAXi);
    //Derivative:
    if(start){
        der = 0.0f;
        start = false;
    }else if(deltaT == 0){
        der = MAXd;
    }else{
        der = constrainPID(kd*1000*(error - errorAnt)/deltaT, MAXd);
    }
    
    errorAnt = error;
    count = (count+1)%BUFFER_SIZE;
    return constrainPID(prop + integ + der, MAXall);
}
