#include "Kontrol.hpp"

# define PI 3.14159265


double calc_Angle(double FFT_output1, double FFT_output2){ // input from the FFT
	double H = sqrt(pow(FFT_output1,2)+pow(FFT_output2,2)); // calculating the lengt of the H-field
	std::cout << H;
	std::cout << "\n";
	double angle = FFT_output1/H_calc; // calculating the input for acos func.
	std::cout << "acos input: ";
	std::cout << angle;
	double alpha = acos(angle)*180.0/PI; // the angle we want to find
	std::cout << " alpha: ";
	std::cout << alpha;
	return alpha;
}



