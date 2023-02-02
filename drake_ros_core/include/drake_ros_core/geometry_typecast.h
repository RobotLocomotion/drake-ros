#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <drake/common/eigen_types.h>

namespace py = pybind11;

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

    // Typecasting between numpy.array (Python) and
    // Eigen::Vector3d (C++)
    template <> struct type_caster<Eigen::Vector3d> {
    public:
        PYBIND11_TYPE_CASTER(Eigen::Vector3d,
            _("numpy.array"));

        // Convert from python numpy.array to
        // C ++ Eigen::Vector3d
        bool load(handle src, bool) {
          object source = reinterpret_borrow<object>(src);

          py::str shape = source.attr("shape");
          if (std::string(shape) != "(3, 1)") {
            return false;
          }

          double* buffer = (double*)static_cast<py::array_t<double>>(source).request().ptr;

          value[0] = buffer[0];
          value[1] = buffer[1];
          value[2] = buffer[2];

          return true;
        }

        // Converting from C++ Eigen::Vector3d to
        // Python numpy.array
        static handle cast(Eigen::Vector3d src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          py::array_t<double> vector = py::array_t<double>({3,1});
          double* buffer = (double*)vector.request().ptr;
          buffer[0] = src(0,0);
          buffer[1] = src(1,0);
          buffer[2] = src(2,0);

          object instance = module::import("numpy").attr("array")(vector);

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

    // Typecasting between numpy.array (Python) and
    // drake::Vector6d (C++)
    template <> struct type_caster<drake::Vector6d> {
    public:
        PYBIND11_TYPE_CASTER(drake::Vector6d,
            _("numpy.array"));

        // Convert from python numpy.array to
        // C ++ drake::Vector6d
        bool load(handle src, bool) {
          object source = reinterpret_borrow<object>(src);

          py::str shape = source.attr("shape");
          if (std::string(shape) != "(6, 1)") {
            return false;
          }

          double* buffer = (double*)static_cast<py::array_t<double>>(source).request().ptr;

          value[0] = buffer[0];
          value[1] = buffer[1];
          value[2] = buffer[2];
          value[3] = buffer[3];
          value[4] = buffer[4];
          value[5] = buffer[5];

          return true;
        }

        // Converting from C++ drake::Vector6d to
        // Python numpy.array
        static handle cast(drake::Vector6d src,
            return_value_policy policy, handle parent) {
          (void)policy;
          (void)parent;

          py::array_t<double> vector = py::array_t<double>({6,1});
          double* buffer = (double*)vector.request().ptr;
          buffer[0] = src(0,0);
          buffer[1] = src(1,0);
          buffer[2] = src(2,0);
          buffer[3] = src(3,0);
          buffer[4] = src(4,0);
          buffer[5] = src(5,0);

          object instance = module::import("numpy").attr("array")(vector);

          instance.inc_ref();
          return instance;
        }
    };

}} // namespace PYBIND11_NAMESPACE::detail
