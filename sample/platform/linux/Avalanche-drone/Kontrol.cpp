#include "Kontrol.hpp"

# define PI 3.14159265

/*
void printhej(){
	char a;
	std::cout << "hejsa";
	//std::cin >> a;	
}
*/

double calc_Angle(double FFT_output1, double FFT_output2){
	double H = sqrt(pow(FFT_output1,2)+pow(FFT_output2,2));
	double H_calc = abs(H);
	std::cout << H_calc;
	std::cout << "\n";
	double angle = FFT_output1/H_calc;
	std::cout << "acos input: ";
	std::cout << angle;
	double alpha = acos(angle)*180.0/PI;
	std::cout << " alpha: ";
	std::cout << alpha;
	return alpha;
}



