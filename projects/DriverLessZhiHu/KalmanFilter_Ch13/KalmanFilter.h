#pragma once
#include "F:/eigen/Eigen/Dense"
//https://zhuanlan.zhihu.com/p/45238681

//====Prediction
//x' = F * x + u        //u=[0, 0, 0, 0]T
//P' = F * P * FT + Q   //P表示系统的不确定程度, P的专业术语叫状态协方差矩阵（state covariance matrix)
                        //Q表示过程噪声（process covariance matrix），即无法用x'=Fx+u表示的噪声     //工程上，将Q设置为单位矩阵

//====Measurement update
//y = z - Hx'           //实际观测到的测量值z与预测值x'之间差值y
//S = H * P' * HT + R   //R是测量噪声矩阵（measurement covariance matrix），这个表示的是测量值与真值之间的差值  //厂家会提供该值
                        //S是一个临时变量
//K = P' * HT * S^1     //卡尔曼增益K（Kalman Gain）    //y值的权值
//完成了卡尔曼滤波器的闭环
//x = x' + K * y        //完成了当前状态向量x的更新 = 预测值 + 测量值 + 噪声
//P = (I - K * H) * P'  //更新了系统的不确定度P

class KalmanFilter {
public:
    KalmanFilter() {
        is_initialized_ = false;
    }

    ~KalmanFilter() = default;

    void Initialization(Eigen::VectorXd x_in) {
        x_ = x_in;
    }

    bool IsInitialized() {
        return is_initialized_;
    }

    Eigen::VectorXd GetX() {
        return x_;
    }

    void SetF(Eigen::MatrixXd F_in) {
        F_ = F_in;
    }

    void SetP(Eigen::MatrixXd P_in) {
        P_ = P_in;
    }

    void SetQ(Eigen::MatrixXd Q_in) {
        Q_ = Q_in;
    }

    void SetH(Eigen::MatrixXd H_in) {
        H_ = H_in;
    }

    void SetR(Eigen::MatrixXd R_in) {
        R_ = R_in;
    }

    void Prediction() {
        x_ = F_ * x_;           //u=[0, 0, 0, 0]T
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
    }

    void MeasurementUpdate(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + (K * y);
        size_t size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K * H_) * P_;
    }

private:
    bool is_initialized_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};
