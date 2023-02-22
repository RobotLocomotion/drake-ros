#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <drake/common/eigen_types.h>

namespace py = pybind11;

// TODO(Aditya):Use serialization-deserialization based generic typecasting for
// ROS msgs.
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

    // Typecasting between geometry_msgs.msg.Vector3 (Python) and
    // geometry_msgs::msg::Vector3 (C++)
    template <> struct type_caster<geometry_msgs::msg::Vector3> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Vector3,
            _("geometry_msgs.msg.Vector3"));

        // Convert from python geometry_msgs.msg.Vector3 to
        // C ++ geometry_msgs::msg::Vector3
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Vector3");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.x = source.attr("x").cast<double>();
          value.y = source.attr("y").cast<double>();
          value.z = source.attr("z").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Vector3 to
        // Python geometry_msgs.msg.Vector3
        static handle cast(geometry_msgs::msg::Vector3 src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Vector3")();
          instance.attr("x") = src.x;
          instance.attr("y") = src.y;
          instance.attr("z") = src.z;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Twist (Python) and
    // geometry_msgs::msg::Twist (C++)
    template <> struct type_caster<geometry_msgs::msg::Twist> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Twist,
            _("geometry_msgs.msg.Twist"));

        // Convert from python geometry_msgs.msg.Twist to
        // C ++ geometry_msgs::msg::Twist
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Twist");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.linear.x = source.attr("linear").attr("x").cast<double>();
          value.linear.y = source.attr("linear").attr("y").cast<double>();
          value.linear.z = source.attr("linear").attr("z").cast<double>();
          value.angular.x = source.attr("angular").attr("x").cast<double>();
          value.angular.y = source.attr("angular").attr("y").cast<double>();
          value.angular.z = source.attr("angular").attr("z").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Twist to
        // Python geometry_msgs.msg.Twist
        static handle cast(geometry_msgs::msg::Twist src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Twist")();
          instance.attr("linear").attr("x") = src.linear.x;
          instance.attr("linear").attr("y") = src.linear.y;
          instance.attr("linear").attr("z") = src.linear.z;
          instance.attr("angular").attr("x") = src.angular.x;
          instance.attr("angular").attr("y") = src.angular.y;
          instance.attr("angular").attr("z") = src.angular.z;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Accel (Python) and
    // geometry_msgs::msg::Accel (C++)
    template <> struct type_caster<geometry_msgs::msg::Accel> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Accel,
            _("geometry_msgs.msg.Accel"));

        // Convert from python geometry_msgs.msg.Accel to
        // C ++ geometry_msgs::msg::Accel
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Accel");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.linear.x = source.attr("linear").attr("x").cast<double>();
          value.linear.y = source.attr("linear").attr("y").cast<double>();
          value.linear.z = source.attr("linear").attr("z").cast<double>();
          value.angular.x = source.attr("angular").attr("x").cast<double>();
          value.angular.y = source.attr("angular").attr("y").cast<double>();
          value.angular.z = source.attr("angular").attr("z").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Accel to
        // Python geometry_msgs.msg.Accel
        static handle cast(geometry_msgs::msg::Accel src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Accel")();
          instance.attr("linear").attr("x") = src.linear.x;
          instance.attr("linear").attr("y") = src.linear.y;
          instance.attr("linear").attr("z") = src.linear.z;
          instance.attr("angular").attr("x") = src.angular.x;
          instance.attr("angular").attr("y") = src.angular.y;
          instance.attr("angular").attr("z") = src.angular.z;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Wrench (Python) and
    // geometry_msgs::msg::Wrench (C++)
    template <> struct type_caster<geometry_msgs::msg::Wrench> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Wrench,
            _("geometry_msgs.msg.Wrench"));

        // Convert from python geometry_msgs.msg.Wrench to
        // C ++ geometry_msgs::msg::Wrench
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Wrench");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.force.x = source.attr("force").attr("x").cast<double>();
          value.force.y = source.attr("force").attr("y").cast<double>();
          value.force.z = source.attr("force").attr("z").cast<double>();
          value.torque.x = source.attr("torque").attr("x").cast<double>();
          value.torque.y = source.attr("torque").attr("y").cast<double>();
          value.torque.z = source.attr("torque").attr("z").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Wrench to
        // Python geometry_msgs.msg.Wrench
        static handle cast(geometry_msgs::msg::Wrench src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Wrench")();
          instance.attr("force").attr("x") = src.force.x;
          instance.attr("force").attr("y") = src.force.y;
          instance.attr("force").attr("z") = src.force.z;
          instance.attr("torque").attr("x") = src.torque.x;
          instance.attr("torque").attr("y") = src.torque.y;
          instance.attr("torque").attr("z") = src.torque.z;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Pose (Python) and
    // geometry_msgs::msg::Pose (C++)
    template <> struct type_caster<geometry_msgs::msg::Pose> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Pose,
            _("geometry_msgs.msg.Pose"));

        // Convert from python geometry_msgs.msg.Pose to
        // C ++ geometry_msgs::msg::Pose
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Pose");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.position.x = source.attr("position").attr("x").cast<double>();
          value.position.y = source.attr("position").attr("y").cast<double>();
          value.position.z = source.attr("position").attr("z").cast<double>();

          value.orientation.x = source.attr("orientation").attr("x").cast<double>();
          value.orientation.y = source.attr("orientation").attr("y").cast<double>();
          value.orientation.z = source.attr("orientation").attr("z").cast<double>();
          value.orientation.w = source.attr("orientation").attr("w").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Pose to
        // Python geometry_msgs.msg.Pose
        static handle cast(geometry_msgs::msg::Pose src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Pose")();
          instance.attr("position").attr("x") = src.position.x;
          instance.attr("position").attr("y") = src.position.y;
          instance.attr("position").attr("z") = src.position.z;

          instance.attr("orientation").attr("x") = src.orientation.x;
          instance.attr("orientation").attr("y") = src.orientation.y;
          instance.attr("orientation").attr("z") = src.orientation.z;
          instance.attr("orientation").attr("w") = src.orientation.w;

          instance.inc_ref();
          return instance;
        }
    };

    // Typecasting between geometry_msgs.msg.Transform (Python) and
    // geometry_msgs::msg::Transform (C++)
    template <> struct type_caster<geometry_msgs::msg::Transform> {
    public:
        PYBIND11_TYPE_CASTER(geometry_msgs::msg::Transform,
            _("geometry_msgs.msg.Transform"));

        // Convert from python geometry_msgs.msg.Transform to
        // C ++ geometry_msgs::msg::Transform
        bool load(handle src, bool) {
          handle cls = module::import("geometry_msgs.msg").attr("Transform");
          if (!isinstance(src, cls)) {
            return false;
          }
          object source = reinterpret_borrow<object>(src);

          value.translation.x = source.attr("translation").attr("x").cast<double>();
          value.translation.y = source.attr("translation").attr("y").cast<double>();
          value.translation.z = source.attr("translation").attr("z").cast<double>();

          value.rotation.x = source.attr("rotation").attr("x").cast<double>();
          value.rotation.y = source.attr("rotation").attr("y").cast<double>();
          value.rotation.z = source.attr("rotation").attr("z").cast<double>();
          value.rotation.w = source.attr("rotation").attr("w").cast<double>();

          return true;
        }

        // Converting from C++ geometry_msgs::msg::Transform to
        // Python geometry_msgs.msg.Transform
        static handle cast(geometry_msgs::msg::Transform src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          object instance = module::import("geometry_msgs.msg").attr("Transform")();
          instance.attr("translation").attr("x") = src.translation.x;
          instance.attr("translation").attr("y") = src.translation.y;
          instance.attr("translation").attr("z") = src.translation.z;

          instance.attr("rotation").attr("x") = src.rotation.x;
          instance.attr("rotation").attr("y") = src.rotation.y;
          instance.attr("rotation").attr("z") = src.rotation.z;
          instance.attr("rotation").attr("w") = src.rotation.w;

          instance.inc_ref();
          return instance;
        }
    };

}} // namespace PYBIND11_NAMESPACE::detail
