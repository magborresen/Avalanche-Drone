#ifndef IIRFilter_H
#define IIRFilter_H

class IIRFilter{
    public:
        double filter(double input); //function to be called by program take the next value as input and outputs the "filtered" valua at that point n
        void resetFilter();
    private:
        int numberofSections = 5;
        /*
            To use the filter simply input a value x[n] and the output will be the "filtered" value Y[n]
            Y = filter(X[n]);
        */
        //numerator from the transferfunction generated by matlab
        const double NUM[5][3] = {
            {1, 0,-1},
            {1, 0,-1},
            {1, 0,-1},
            {1, 0,-1},
            {1, 0,-1},
        };
        //denumerator from the transferfunction generated by matlab
        const double DENUM[5][3] = {
            {1,  -1.149714982285,  0.9993504168974},
            {1,  -1.152982111239,  0.9993513303548}, 
            {1,  -1.149735606289,  0.998300717812},
            {1,  -1.151754269744,  0.9983021942678},
            {1,  -1.150514714051,  0.9979009042415},
        };
        //Gain from the transferfunction generated by matlab
        const double sectionGain[5] = {
            0.001050309060505,                                                         
            0.001050309060505,                                                        
            0.00104975809907,                                                         
            0.00104975809907,                                                         
            0.001049547879256,                                                         
        };
        double section(int sec, double in); //function used for calculating eat 2. order section see https://se.mathworks.com/help/signal/ref/dfilt.df2sos.html
        double w[5][3] = {0}; //set all w to zero else they will be undetermined ie. have random values
};

#endif
