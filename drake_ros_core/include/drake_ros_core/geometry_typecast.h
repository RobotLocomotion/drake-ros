#include <pybind11/pybind11.h>

#include <geometry_msgs/msg/quaternion.hpp>

namespace PYBIND11_NAMESPACE { namespace detail {
    template <> struct type_caster<geometry_msgs::msg::Quaternion> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Quaternion,
            _("geometry_msgs.msg.Quaternion"));

        // Convert from python geometry_msgs.msg.Quaternion to
        // C ++ geometry_msgs::msg::Quaternion
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Quaternion");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.x = source.attr("x").cast<double>();
          value.y = source.attr("y").cast<double>();
          value.z = source.attr("z").cast<double>();
          value.w = source.attr("w").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Quaternion to
        // python geometry_msgs.msg.Quaternion
        static handle cast(geometry_msgs::msg::Quaternion src, return_value_policy policy, handle parent) {
          (void)src;
          (void)policy;
          (void)parent;
          Py_RETURN_NOTIMPLEMENTED;
        }
    };
}} // namespace PYBIND11_NAMESPACE::detail
