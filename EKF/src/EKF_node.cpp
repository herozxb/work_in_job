///
/// @author Mohanad Youssef
/// @file main.cpp
///

#include <iostream>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>

#define CVPLOT_HEADER_ONLY
#include <CvPlot/cvplot.h>

#include "../include/EKF/types.h"
#include "../include/EKF/kalman_filter.h"

using namespace cv;
using namespace std;

static constexpr size_t DIM_X{ 1 };
static constexpr size_t DIM_Z{ 2 };

static kf::KalmanFilter<DIM_X, DIM_Z> kalmanfilter;
void executeCorrectionStep();

const int N = 200; // Number of points

int main(int argc, char ** argv)
{   
    executeCorrectionStep( );   
    return 0;
}

vector<double> generate_signal( double bias, double variance )
{
    
    // Define sine wave parameters
    const double freq = 1.0; // Frequency in Hz
    const double amplitude = 1.0; // Amplitude of sine wave
    const double phase = 0;//M_PI / 4; // Phase of sine wave

    // Define noise parameters
    const double mean = 0.0; // Mean of noise
    //const double variance = 0.1; // Variance of noise

    // Define time vector
    // const int N = 1000; // Number of points
    const double dt = 0.01; // Time step
    vector<double> t(N);
    for (int i = 0; i < N; i++) {
        t[i] = i * dt;
    }

    // Generate sine wave with added noise
    vector<double> x(N);
    default_random_engine generator;
    normal_distribution<double> distribution(mean, sqrt(variance));
    for (int i = 0; i < N; i++) {
        x[i] = amplitude * sin(2.0 * M_PI * freq * t[i] + phase) + distribution(generator) + bias ;
    }

    // Print generated points
    for (int i = 0; i < N; i++) {
        cout << t[i] << "," << x[i] << endl;
    }

    cout<<"==================================================="<<endl;
    
    return x;
}

void executeCorrectionStep()
{

    kalmanfilter.vector_x() << 1.0F;
    kalmanfilter.matrix_P() << 1.0F;

    vector<double> x_1(N);
    vector<double> x_2(N);
    
    double bias_1 = -3;
    double bias_2 = 3;
    double variance_1 = 0.1;
    double variance_2 = 0.5;
    
    x_1 = generate_signal(  bias_1, variance_1 );
    x_2 = generate_signal(  bias_2, variance_2 );
    
    kf::Matrix<1, 1>  matrix_F{ kf::Matrix<1, 1>::Identity() };  // 1x1
    matrix_F << 1.0F;
    
    kf::Matrix<1, 1>  matrix_Q{ kf::Matrix<1, 1>::Identity() };  // 1x1
    matrix_Q << 0.05F;
    
    kf::Matrix<2, 2>  matrix_R { kf::Matrix<2, 2>::Identity() }; // 2x2
    matrix_R << variance_1, 0, 0, variance_2;
    
    kf::Matrix<2, 1>  matrix_H { kf::Matrix<2, 1>::Random(2,1) };// 2x1
    matrix_H << 1.0F, 1.0F;
    
    double signal_data_1[N] = {0}; 
    double signal_data_2[N] = {0}; 
    double signal_data_3[N] = {0}; 
    
    for(int i =0; i< N; i++ )
    {

        const kf::Vector<2> vector_z { x_1[i], x_2[i] };
        
    	kalmanfilter.predict(matrix_F, matrix_Q );
    	//kalmanfilter.correct(vector_z, matrix_R, matrix_H);
    	
    	signal_data_1[i] = x_1[i];
    	signal_data_2[i] = x_2[i];
    	
    	if( i < N /2 )
    	{
    	    signal_data_3[i] = kalmanfilter.correct(vector_z, matrix_R, matrix_H);
    	}
    	else
    	{
    	    matrix_R << variance_2, 0, 0, variance_1;
    	    signal_data_3[i] = kalmanfilter.correct(vector_z, matrix_R, matrix_H);
    	}
    }
    std::vector<double> x(N), y1(x.size()), y2(x.size()), y3(x.size());
    const double dt = 0.01; // Time step
    
    for (size_t i = 0; i < x.size(); i++) {
        x[i] = i * dt;
	y1[i] = signal_data_1[i];
	y2[i] = signal_data_2[i];
	y3[i] = signal_data_3[i];
    }
    auto axes = CvPlot::makePlotAxes();


    axes.create<CvPlot::Series>(x, y1, "-r");
    axes.create<CvPlot::Series>(x, y2, "-b");
    axes.create<CvPlot::Series>(x, y3, "-g");       
    
    CvPlot::show("kelman filter", axes);

}

