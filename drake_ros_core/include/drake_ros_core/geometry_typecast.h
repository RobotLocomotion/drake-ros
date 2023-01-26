#include <pybind11/pybind11.h>

#include <geometry_msgs/msg/quaternion.hpp>

namespace PYBIND11_NAMESPACE {
  namespace detail {

    // Typecasting between geometry_msgs.msg.Quaternion (Python) and
    // geometry_msgs::msg::Quaternion (C++)
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
        // Python geometry_msgs.msg.Quaternion
        static handle cast(geometry_msgs::msg::Quaternion src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Quaternion")();
          instance.attr("x") = src.x;
          instance.attr("y") = src.y;
          instance.attr("z") = src.z;
          instance.attr("w") = src.w;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Point (Python) and
    // geometry_msgs::msg::Point (C++)
    template <> struct type_caster<geometry_msgs::msg::Point> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Point,
            _("geometry_msgs.msg.Point"));

        // Convert from python geometry_msgs.msg.Point to
        // C ++ geometry_msgs::msg::Point
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Point");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.x = source.attr("x").cast<double>();
          value.y = source.attr("y").cast<double>();
          value.z = source.attr("z").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Point to
        // Python geometry_msgs.msg.Point
        static handle cast(geometry_msgs::msg::Point src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Point")();
          instance.attr("x") = src.x;
          instance.attr("y") = src.y;
          instance.attr("z") = src.z;

          instance.inc_ref();
          return instance;
        }
    };

}} // namespace PYBIND11_NAMESPACE::detail
