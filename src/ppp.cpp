#include <iostream>
#include <vector>
#include <cmath>

// 数据结构：存储IMU数据
struct IMUData {
    std::vector<double> acceleration;       // 存储加速度数据
    std::vector<double> angular_velocity;   // 存储角速度数据
};

// 数据结构：存储雷达帧数据
struct RadarFrame {
    std::vector<std::vector<double>> points; // 存储3D点云数据
};

// SLAM系统类
class SLAMSystem {
public:
    // 构造函数，初始化SLAM系统
    SLAMSystem(int max_subframes, double max_acc_std)
        : max_subframes(max_subframes), max_acc_std(max_acc_std) {}

    // 划分雷达帧为多个雷达子帧
    std::vector<RadarFrame> divideRadarFrame(const IMUData& imuData) {
        // 计算加速度的标准差
        double acc_std = calculateStd(imuData.acceleration);

        // 使用比例控制器计算子帧数量
        int subframesCount = static_cast<int>(max_subframes * acc_std / max_acc_std);

        // 避免子帧数量超过最大限制
        subframesCount = std::min(subframesCount, max_subframes);

        // 划分雷达帧
        int subframeSize = imuData.points.size() / subframesCount;
        for (int i = 0; i < subframesCount; ++i) {
            int startIdx = i * subframeSize;
            int endIdx = (i + 1) * subframeSize;
            RadarFrame subframe;
            subframe.points = {imuData.points.begin() + startIdx, imuData.points.begin() + endIdx};
            subframes.push_back(subframe);
        }

        return subframes;
    }

private:
    // 计算一维数据的标准差
    double calculateStd(const std::vector<double>& data) {
        // 计算均值
        double mean = 0.0;
        for (double value : data) {
            mean += value;
        }
        mean /= data.size();

        // 计算方差
        double variance = 0.0;
        for (double value : data) {
            variance += pow(value - mean, 2);
        }
        variance /= data.size();

        // 返回标准差
        return sqrt(variance);
    }

private:
    int max_subframes; // 最大子帧数
    double max_acc_std; // 加速度的最大标准差
    std::vector<RadarFrame> subframes; // 存储划分得到的雷达子帧
};

int main() {
    // 创建IMU数据
    IMUData imuData;
    imuData.acceleration = {1.0, 2.0, 1.5, 1.2};  // 例子中的加速度数据
    imuData.angular_velocity = {0.1, 0.2, 0.15, 0.12};  // 例子中的角速度数据

    // 创建雷达帧数据
    RadarFrame radarFrame;
    radarFrame.points = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};  // 例子中的雷达帧数据

    // 创建SLAM系统实例
    SLAMSystem slamSystem(3, 1.0);  // 例子中的最大子帧数和加速度最大标准差

    // 划分雷达帧
    std::vector<RadarFrame> subframes = slamSystem.divideRadarFrame(imuData);

    // 显示结果（仅用于演示目的）
    for (int i = 0; i < subframes.size(); ++i) {
        std::cout << "Subframe " << i + 1 << ":" << std::endl;
        for (const auto& point : subframes[i].points) {
            std::cout << "(" << point[0] << ", " << point[1] << ", " << point[2] << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
