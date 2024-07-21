#include "drag.h"

/**
 * A degree-3 polynomial in 2 variables.
 */
typedef struct {
    float p00;
    float p10;
    float p01;
    float p20;
    float p11;
    float p02;
    float p30;
    float p21;
    float p12;
    float p03;
} Cubic2VariablePolynomial;

/**
 * Evaluates a cubic 2 variable polynomial at the given coordinates.
 */
float evaluate_cubic_2_variable(Cubic2VariablePolynomial* poly, float x, float y) {
    return poly->p00 + poly->p10 * x + poly->p01 * y + poly->p20 * x * x + poly->p11 * x * y + poly->p02 * y * y + poly->p30 * x * x * x + poly->p21 * x * x * y + poly->p12 * x * y * y + poly->p03 * y * y * y;
}

float interpolate_drag(float extension, float velocity, float altitude) {
    if (extension < 0) {
        extension = 0;
    } else if (extension > 1) {
        extension = 1;
    }

    Cubic2VariablePolynomial coeffs[11] = {
        { 232.2951, 244.7010, -75.1435, 64.3402, -79.5220, 11.7309, -0.8306, -20.4344, 9.7041, -0.6148 },
        { 235.8993, 249.2100, -76.2767, 65.8251, -81.2931, 12.0289, -0.8408, -21.0236, 9.9787, -0.7853 },
        { 245.6886, 260.2967, -80.2111, 69.3746, -85.4361, 12.5705, -0.6199, -22.1676, 10.4297, -0.6666 },
        { 253.9691, 270.2409, -83.5032, 72.3919, -89.3117, 13.3189, -0.7371, -23.2489, 11.0822, -0.7471 },
        { 263.5127, 280.7695, -86.9338, 75.8929, -93.6884, 14.1591, -0.3771, -24.5324, 11.7445, -0.9399 },
        { 272.4592, 290.5670, -90.1040, 78.8810, -97.0343, 14.5126, -0.1988, -25.6374, 12.0348, -0.7327 },
        { 284.8368, 304.7727, -94.4923, 82.4469, -101.4462, 15.1080, -0.6357, -26.5433, 12.4927, -0.8323 },
        { 296.2638, 317.3919, -98.5746, 86.2809, -106.2663, 15.9556, -0.5224, -28.0106, 13.2557, -0.8541 },
        { 303.1856, 325.1022, -100.9674, 88.7552, -109.3743, 16.5627, -0.4253, -28.9892, 13.8034, -0.9447 },
        { 316.4963, 339.6502, -104.5570, 92.4088, -114.1954, 16.9114, -0.5681, -30.4995, 13.9269, -1.2933 },
        { 340.9146, 367.1520, -114.8622, 101.0439, -123.1214, 18.4047, -0.1879, -31.8071, 15.4422, -1.1456 }
    };

    int index = (int) (extension * 10.0f);
    if (extension * 10.0f == index) {
        return evaluate_cubic_2_variable(&coeffs[index], velocity, altitude);
    }
    float first = evaluate_cubic_2_variable(&coeffs[index], velocity, altitude);
    float second = evaluate_cubic_2_variable(&coeffs[index + 1], velocity, altitude);
    float diff = extension - ((float) ((int) extension));
    return diff * second + (1.0f - diff) * first;
}
