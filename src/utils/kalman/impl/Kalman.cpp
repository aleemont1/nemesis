#include "utils\kalman\Kalman.hpp"

float Kalman(float U){

    static const float R=10;
    static const float H=10;

    static double Q=0;  //Initial estimated covariance
    static double K=0;  //initial Kalman Gain
    static double P=0;  //initial error covariance (must be 0)
    static double U_Filtered=0;

    //First step, update the Kalman Gain
    K = P * H /(H * P * H + R);
    U_Filtered = U_Filtered + K *(U - H * U_Filtered);

    //Next update the error covariance
    P = (1 - K * H)* P + Q;

    return U_Filtered;
}
