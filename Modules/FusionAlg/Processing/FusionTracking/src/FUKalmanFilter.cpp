//
// Created by root on 4/12/21.
//

#include <iostream>
#include <xtensor/xarray.hpp>
#include "FUKalmanFilter.h"
#include <xtensor/xadapt.hpp>
#include <xtensor-blas/xlinalg.hpp>
//
xt::xarray<float> helpers_::reshape_z(xt::xarray<float> z, int dim_z, int ndim) {
    z = xt::atleast_2d(z);
    if (z.shape(1) == dim_z)
        z = xt::transpose(z);
    assert(z.shape(0) == dim_z and z.shape(1) == 1);

    if (ndim == 1)
        z = xt::view(z, xt::all(), 0);
    if (ndim == 0)
        z = z(0, 0);
    return z;

}
//
FUKalmanFilter::FUKalmanFilter(int dim_x, int dim_z, int model_type,int dim_u):
                            _dim_x(dim_x),
                            _dim_z(dim_z),
                            _dim_u(dim_u){
    assert(dim_x >= 1);
    assert(dim_z >= 1);
    assert(dim_u >= 0);
    _x = xt::zeros<float>({dim_x, 1});
    _P = xt::eye(dim_x);
    _Q = xt::eye(dim_x);
    _F = xt::eye<float>(dim_x);
    _H = xt::zeros<float>({dim_z, dim_x});
    _R = xt::eye(dim_z);
    _alpha_sq = 1.;
    _M = xt::zeros<float>({dim_z, dim_z});
    _z = xt::zeros<float>({dim_z, 1}); // transpose
    _K = xt::zeros<float>({dim_x, dim_z});
    _y = xt::zeros<float>({dim_z, 1});
    _S = xt::zeros<float>({dim_z, dim_z});
    _SI = xt::zeros<float>({dim_z, dim_z});
    _I = xt::eye(dim_x);

    if (model_type == 0){//匀速模型
        _F = xt::xarray<float>({
            {1, 0,  0.1, 0,   0, 0},
            {0, 1,  0,   0.1, 0,     0},
            {0, 0,  1,   0,   0,   0},
            {0, 0,  0,   1,   0,     0},
            {0, 0,  0,   0,   1,     0},
            {0, 0,  0,   0,   0,     1}});
        _H = xt::xarray<float>({
            {1, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0}});
        _Q *= 1;
        _R *= 1;
        _P *= 10;
        xt::view(_P, xt::range(2, 4), xt::range(2, 4)) *= 1000;
    }
    if (model_type == 1){//加速模型
        _F = xt::xarray<float>({
            {1, 0,  0.1, 0,   0.005, 0},
            {0, 1,  0,   0.1, 0,     0.005},
            {0, 0,  1,   0,   0.1,   0},
            {0, 0,  0,   1,   0,     0.1},
            {0, 0,  0,   0,   1,     0},
            {0, 0,  0,   0,   0,     1}});
        _H = xt::xarray<float>({
            {1, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0}});
        _Q *= 1;
        _R *= 1;
        _P *= 10;
        xt::view(_P, xt::range(2, 4), xt::range(2, 4)) *= 1000;

    }

    _x_prior = _x;
    _P_prior = _P;
    _x_post = _x;
    _P_post = _P;
}
//
FUKalmanFilter::FUKalmanFilter() {}

FUKalmanFilter::~FUKalmanFilter() {}
//
void FUKalmanFilter::predict(float dt,int model_type) {
    /**
    Predict next state (prior) using the Kalman filter state propagation
    equations.

            Parameters
            ----------

    u : np.array
    Optional control vector. If not `None`, it is multiplied by B
    to create the control input into the system.

            B : np.array(dim_x, dim_z), or None
    Optional control transition matrix; a value of None
    will cause the filter to use `self.B`.

    F : np.array(dim_x, dim_x), or None
    Optional state transition matrix; a value of None
    will cause the filter to use `self.F`.

    Q : np.array(dim_x, dim_x), scalar, or None
    Optional process noise matrix; a value of None will cause the
    filter to use `self.Q`.                 */

    xt::xarray<float> B;
    xt::xarray<float> F;
    xt::xarray<float> Q;
    B = _B;
    Q = _Q;
    if (model_type == 0){
        _F(0,2) = dt;
        _F(1,3) = dt;
        _F(2,4) = 0;
        _F(3,5) = 0;
        _F(0,4) = 0;
        _F(1,5) = 0;
    }
    else{
        _F(0,2) = dt;
        _F(1,3) = dt;
        _F(2,4) = dt;
        _F(3,5) = dt;
        _F(0,4) = 0.5*dt*dt;
        _F(1,5) = 0.5*dt*dt;
    }
    F = _F;

    _x = xt::linalg::dot(F, _x);
    _P = _alpha_sq * xt::linalg::dot(xt::linalg::dot(F, _P), xt::transpose(F)) + Q;
    _x_prior = _x;
    _P_prior = _P;
}

void FUKalmanFilter::update(xt::xarray<float> &z, xt::xarray<float> R, xt::xarray<float> H) {
    if (!z.size()){
        _z = xt::transpose(xt::zeros<float>({1, _dim_z}));
        _x_post = _x;
        _P_post = _P;
        _y = xt::zeros<float>({_dim_z, 1});
        return;
    }
    xt::xarray<float> z_shape = xt::adapt(z.shape());
    z = helpers_::reshape_z(z, _dim_z, z_shape.size());

    if (!R.size())
        R = _R;
    if (!H.size())
        H = _H;
    _y = z - xt::linalg::dot(H, _x);
    xt::xarray<float> PHT = xt::linalg::dot(_P, xt::transpose(H));

    _S = xt::linalg::dot(H, PHT) + R;

    _SI(0,0) = 1/_S(0,0);
    _SI(1,1) = 1/_S(1,1);

    _K = xt::linalg::dot(PHT, _SI);

    _x = _x + xt::linalg::dot(_K, _y);

    xt::xarray<float> I_KH = _I - xt::linalg::dot(_K, H);

    _P = xt::linalg::dot(xt::linalg::dot(I_KH, _P), xt::transpose(I_KH)) +
            xt::linalg::dot(xt::linalg::dot(_K, R), xt::transpose(_K));

    _z = z;
    _x_post = _x;
    _P_post = _P;
}
