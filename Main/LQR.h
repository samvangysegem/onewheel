
#ifndef _LQR_h_
#define _LQR_h_

class LQR {

    public:
        LQR(); // Constructor
    
    public:
        // Set/Get methods
        
    private:
        // Model and control variables
        // Solution of the Ricattian is rewritten as u = V*(x-ksi)+W where ksi is the desired state
        float V[4];


};

#endif