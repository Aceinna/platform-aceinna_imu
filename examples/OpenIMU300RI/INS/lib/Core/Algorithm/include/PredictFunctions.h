#ifndef EKF_ALGORITHM_H
#define EKF_ALGORITHM_H

void EKF_PredictionStage(real *filteredAccel);
void GenerateProcessCovariance(void);
void GenerateProcessJacobian(void);

// Size of EKF matrices
#define  ROWS_IN_P  16
#define  COLS_IN_P  16

#define  ROWS_IN_F  16
#define  COLS_IN_F  16

#endif /* EKF_ALGORITHM_H */
