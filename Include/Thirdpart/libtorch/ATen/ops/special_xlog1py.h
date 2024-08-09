#pragma once

// @generated by tools/codegen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>



#include <ATen/ops/special_xlog1py_ops.h>

namespace at {


// aten::special_xlog1py(Tensor self, Tensor other) -> Tensor
TORCH_API inline at::Tensor special_xlog1py(const at::Tensor & self, const at::Tensor & other) {
    return at::_ops::special_xlog1py::call(self, other);
}

// aten::special_xlog1py.self_scalar(Scalar self, Tensor other) -> Tensor
TORCH_API inline at::Tensor special_xlog1py(const at::Scalar & self, const at::Tensor & other) {
    return at::_ops::special_xlog1py_self_scalar::call(self, other);
}

// aten::special_xlog1py.other_scalar(Tensor self, Scalar other) -> Tensor
TORCH_API inline at::Tensor special_xlog1py(const at::Tensor & self, const at::Scalar & other) {
    return at::_ops::special_xlog1py_other_scalar::call(self, other);
}

// aten::special_xlog1py.out(Tensor self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_out(at::Tensor & out, const at::Tensor & self, const at::Tensor & other) {
    return at::_ops::special_xlog1py_out::call(self, other, out);
}

// aten::special_xlog1py.out(Tensor self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_outf(const at::Tensor & self, const at::Tensor & other, at::Tensor & out) {
    return at::_ops::special_xlog1py_out::call(self, other, out);
}

// aten::special_xlog1py.self_scalar_out(Scalar self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_out(at::Tensor & out, const at::Scalar & self, const at::Tensor & other) {
    return at::_ops::special_xlog1py_self_scalar_out::call(self, other, out);
}

// aten::special_xlog1py.self_scalar_out(Scalar self, Tensor other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_outf(const at::Scalar & self, const at::Tensor & other, at::Tensor & out) {
    return at::_ops::special_xlog1py_self_scalar_out::call(self, other, out);
}

// aten::special_xlog1py.other_scalar_out(Tensor self, Scalar other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_out(at::Tensor & out, const at::Tensor & self, const at::Scalar & other) {
    return at::_ops::special_xlog1py_other_scalar_out::call(self, other, out);
}

// aten::special_xlog1py.other_scalar_out(Tensor self, Scalar other, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & special_xlog1py_outf(const at::Tensor & self, const at::Scalar & other, at::Tensor & out) {
    return at::_ops::special_xlog1py_other_scalar_out::call(self, other, out);
}

}
