# Kalman Filter Implementation

This project implements a Kalman Filter for state estimation. The code provides a `KalmanFilter` class that can be used to predict and correct states in a dynamic system using the Kalman filter algorithm.

## File Structure

- `kalmanfilter.h`: Header file containing the definition of the `KalmanFilter` class and its member functions.

## Class: `KalmanFilter`

The `KalmanFilter` class is designed to estimate the state of a system over time, using a series of measurements observed over time, containing statistical noise and other inaccuracies.

### Private Members

- Matrices for state transition (`A`), control input (`B`), measurement (`C`), direct transition (`D`), process noise covariance (`Q`), measurement noise covariance (`R`), and Kalman gain (`gainK`).
- Vectors for state prediction (`predictX`), error (`errorY`), input (`U`), measurement (`Y`), and identity matrix (`I33`).
- Covariance matrices (`P`, `P_old`, `P_new`).
- Intermediate matrices for storing results (`resultX`, `resultY`).
- Arrays for updated input and measurement.
- An array for storing the estimate state.
  
### Private Methods

- `doKalman_gain()`: Calculates the Kalman gain.
- `doCorrect_p()`: Updates the error covariance matrix after measurement update.
- `doPredict_y()`: Predicts the output of the system.
- `doCorrect()`: Corrects the state estimate using the Kalman gain.
- `doPredict_x()`: Predicts the next state of the system.
- `doPredict_p()`: Predicts the error covariance matrix.
- `doResult()`: Stores the final result after prediction and correction.
- `run()`: Runs the complete Kalman filter algorithm.

### Public Methods

- `KalmanFilter(float *_A_data, float *_B_data, float *_C_data, float *_Q_data, float *_R_data)`: Constructor to initialize the Kalman filter with given matrices.
- `begin()`: Initializes the Kalman filter.
- `float *Compute(double _q, float _Vin)`: Computes the state estimate based on the input measurements.

## Usage

To use the `KalmanFilter` class, follow these steps:

1. **Include the Header File**: Ensure the `kalmanfilter.h` file is included in your project.
   ```cpp
   #include "kalmanfilter.h"
   ```

2. **Initialize the Kalman Filter**: Create an instance of the `KalmanFilter` class and initialize it with the required matrices.
   ```cpp
   float A_data[] = { /* Your data here [4 x 4] */ };
   float B_data[] = { /* Your data here [4 x 1]*/ };
   float C_data[] = { /* Your data here [1 x 4]*/ };
   float Q_data[] = { /* Your data here [scalar]*/ };
   float R_data[] = { /* Your data here [scalar]*/ };

   KalmanFilter kf(A_data, B_data, C_data, Q_data, R_data);
   kf.begin();
   ```

3. **Compute State Estimate**: Use the `Compute` method to get the state estimate based on input measurements.
   ```cpp
   double q = /* Your measurement */;
   float Vin = /* Your input */;

   float *state = kf.Compute(q, Vin);
   ```

## Example

Here is an example of how to use the `KalmanFilter` class in a main function:

```cpp
#include <iostream>
#include "kalmanfilter.h"

int main() {
    float A_data[] = { /* Your data here */ };
    float B_data[] = { /* Your data here */ };
    float C_data[] = { /* Your data here */ };
    float Q_data[] = { /* Your data here */ };
    float R_data[] = { /* Your data here */ };

    KalmanFilter kf(A_data, B_data, C_data, Q_data, R_data);
    kf.begin();

    double q = /* Your measurement */;
    float Vin = /* Your input */;

    float *state = kf.Compute(q, Vin);

    std::cout << "Estimated state: ";
    for(int i = 0; i < 4; i++) {
        std::cout << state[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
```
