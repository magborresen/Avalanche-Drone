#include "IIRFilter.hpp"

double IIRFilter::filter(double input){
    //first section
    double y;
    y = section(0, input);
    //loops throught each section giving the output of one section as input to the next
    for (int i = 1; i < numberofSections; i++)
    {
        y = section(i, y);
    }
    //return the final value,
    return y;
}

double IIRFilter::section(int sec, double in){
    // calculate the difference equation 
    //  w = x*gain - w[n-1]*a1 - w[n-2]*a2
    //  y = w[n]*b0 + w[n-1]*b1 + w[n-2]*b2
    // see http://www.ece.northwestern.edu/local-apps/matlabhelp/toolbox/signal/dfilt.df2sos.html for signal diagram

    w[sec][0] = in*sectionGain[sec] - w[sec][1]*DENUM[sec][1] - w[sec][2]*DENUM[sec][2];
    double y = w[sec][0] * NUM[sec][0] + w[sec][1]*NUM[sec][1] + w[sec][2]*NUM[sec][2];
    w[sec][2] = w[sec][1]; //shift w[1] into w[2] so ready for next step
    w[sec][1] = w[sec][0]; //shift w[0] into w[1] so ready for next step
    return y;
}

void IIRFilter::resetFilter(){
    for(int i = 0 ; i< numberofSections ; i++){
        for(int j = 0 ; j < 3 ; j++){
            w[i][j] = 0;
        }
    }
}
