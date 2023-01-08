//cosInterp
//  Cosine interpolation
//  t in [0,1]

#include <cmath>

double cosInterp(double x0, double xg, double t){
    // cosInterp: Cosine interpolation
    // t in [0,1]
    return x0 + (xg-x0)/2*(1 - std::cos(M_PI*t));
}