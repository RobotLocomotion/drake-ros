#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>

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

}} // namespace PYBIND11_NAMESPACE::detail
