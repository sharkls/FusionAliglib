#include <iostream>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xmath.hpp>
#include <cmath>

class StandardScaler {
private:
    xt::xarray<float> mean;
    xt::xarray<float> std;

public:
    //StandardScaler类的fit函数计算平均值和标准差，transform函数执行标准化操作
    void fit(const xt::xarray<float>& X) {
        size_t numFeatures = X.shape()[1];
        mean.resize({numFeatures});
        std.resize({numFeatures});

        xt::xarray<float> sum = xt::sum(X, {0});
        mean = sum / X.shape()[0];

        xt::xarray<float> squaredSum = xt::sum(xt::pow(X - mean, 2), {0});
        std = xt::sqrt(squaredSum / X.shape()[0]);

        // Avoid dividing by zero
        //由于在计算标准差时，可能导致除以零或者出现无效的结果，从而得到NaN值。为了解决这个问题，可以检查标准差是否为零，并在计算过程中避免除以零
        for (auto& value : std) {
            if (std::abs(value) < 1e-8) {
                value = 1.0;  // Set the value to 1 to avoid NaN
            }
        }
    }

    xt::xarray<float> transform(const xt::xarray<float>& X) {
        return (X - mean) / std;
    }
};