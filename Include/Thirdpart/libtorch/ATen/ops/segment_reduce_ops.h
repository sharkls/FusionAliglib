#pragma once

// @generated by tools/codegen/gen.py from Operator.h

#include <c10/core/QScheme.h>
#include <tuple>
#include <vector>


// Forward declarations of any types needed in the operator signatures.
// We can't directly include these classes because it will cause circular include dependencies.
// This file is included by TensorBody.h, which defines the Tensor class.
namespace c10 {

template<typename T>
class optional;
template<typename T>
class List;
class Stream;
class Scalar;
struct Storage;
struct TensorOptions;

}

namespace at {

class Tensor;
struct Dimname;
struct Generator;
using TensorList = c10::ArrayRef<Tensor>;
using DimnameList = c10::ArrayRef<Dimname>;
using c10::Stream;
using c10::Storage;
using c10::QScheme;
using c10::Scalar;
using c10::TensorOptions;

namespace _ops {


struct TORCH_API segment_reduce {
  using schema = at::Tensor (const at::Tensor &, c10::string_view, const c10::optional<at::Tensor> &, const c10::optional<at::Tensor> &, int64_t, bool, const c10::optional<at::Scalar> &);
  using ptr_schema = schema*;
  // See Note [static constexpr char* members for windows NVCC]
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(name, "aten::segment_reduce")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(overload_name, "")
  STATIC_CONSTEXPR_STR_INL_EXCEPT_WIN_CUDA(schema_str, "segment_reduce(Tensor data, str reduce, *, Tensor? lengths=None, Tensor? indices=None, int axis=0, bool unsafe=False, Scalar? initial=None) -> Tensor")
  static at::Tensor call(const at::Tensor & data, c10::string_view reduce, const c10::optional<at::Tensor> & lengths, const c10::optional<at::Tensor> & indices, int64_t axis, bool unsafe, const c10::optional<at::Scalar> & initial);
  static at::Tensor redispatch(c10::DispatchKeySet dispatchKeySet, const at::Tensor & data, c10::string_view reduce, const c10::optional<at::Tensor> & lengths, const c10::optional<at::Tensor> & indices, int64_t axis, bool unsafe, const c10::optional<at::Scalar> & initial);
};

}} // namespace at::_ops
