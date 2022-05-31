#include <iostream>

#include "KalmanFilter.h"

bool GetLidarData(double &x, double &y, double &t) {
    static int count =0;
    if(count == 0) {
        x = 1.0;
        y = 2.0;
        t = 1.0;
        ++count;
        return true;
    } else if(count == 1) {
        x = 2.1;
        y = 4.2;
        t = 2.3;
        ++count;
        return true;
    }
    return false;
}

int main() {
    double m_x = 0.0, m_y=0.0;
    double last_timestamp = 0.0, now_timestamp = 0.0;
    KalmanFilter kf;

    while(GetLidarData(m_x, m_y, now_timestamp)) {
        if(!kf.IsInitialized()) {
            last_timestamp = now_timestamp;
            Eigen::VectorXd x_in(4, 1);
            x_in << m_x, m_y, 0.0, 0.0;
            kf.Initialization(x_in);

            Eigen::MatrixXd p_in(4, 4);
            p_in << 1.0, 0.0, 0.0,   0.0,
                    0.0, 1.0, 0.0,   0.0,
                    0.0, 0.0, 100.0, 0.0,
                    0.0, 0.0, 0.0,   100.0;
            kf.SetP(p_in);

#if 0       //Same
            Eigen::MatrixXd q_in(4, 4);
#else
            Eigen::Matrix4d q_in;
#endif
            q_in << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
            kf.SetQ(p_in);

            Eigen::MatrixXd h_in(2, 4);
            h_in << 1.0, 0.0, 0.0,   0.0,
                    0.0, 1.0, 0.0,   0.0;
            kf.SetH(h_in);

            Eigen::MatrixXd r_in(2, 2);
            r_in << 0.0225, 0.0,
                    0.0,    0.0225;
            kf.SetR(r_in);
            //continue????
        }

        double dt = now_timestamp - last_timestamp;
        last_timestamp = now_timestamp;
        Eigen::MatrixXd f_in(4, 4);
        f_in << 1.0, 0.0, dt, 0.0,
                0.0, 1.0, 0.0, dt,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        kf.SetF(f_in);
        kf.Prediction();

        Eigen::VectorXd z(2, 1);
        z << m_x, m_y;
        kf.MeasurementUpdate(z);

        Eigen::VectorXd x = kf.GetX();
        std::cout << "x: " << x(0) << " y: " << x(1) << std::endl;
    }
}