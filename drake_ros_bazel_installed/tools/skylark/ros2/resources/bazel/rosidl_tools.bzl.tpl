# -*- python -*-

load("@REPOSITORY_ROOT@:distro.bzl", "AVAILABLE_TYPESUPPORTS")
load("@python_dev//:version.bzl", "PYTHON_EXTENSION_SUFFIX")

def _as_idl_tuple(file):
    path, parent, base = file.short_path.rsplit("/", 2)
    if parent not in ("msg", "srv", "action"):
        fail("Interface parent folder must be one of: 'msg', 'srv', 'action'")
    return "{}:{}/{}".format(path, parent, base)

def _as_include_flag(file):
    return "-I" + file.short_path.rsplit("/", 3)[0]

def _rosidl_generate_genrule_impl(ctx):
    args = ctx.actions.args()
    args.add("generate")
    output_path = "{}/{}".format(
        ctx.var["GENDIR"],
        ctx.label.package)
    if ctx.attr.output_dir:
        output_path = "{}/{}".format(
            output_path, ctx.attr.output_dir)
    args.add("--output-path", output_path)
    for type in ctx.attr.types:
        args.add("--type", type)
    for typesupport in ctx.attr.typesupports:
        args.add("--type-support", typesupport)
    args.add_all(ctx.files.includes, map_each=_as_include_flag, uniquify=True)
    args.add(ctx.attr.package)
    args.add_all(ctx.files.interfaces, map_each=_as_idl_tuple)
    inputs, input_manifests = ctx.resolve_tools(tools = [ctx.attr._tool])
    inputs = inputs.to_list() + ctx.files.interfaces + ctx.files.includes
    ctx.actions.run(
        executable = ctx.executable._tool,
        arguments = [args], inputs = inputs,
        input_manifests = input_manifests,
        outputs = ctx.outputs.generated_sources,
    )

rosidl_generate_genrule = rule(
    attrs = dict(
        generated_sources = attr.output_list(mandatory = True),
        types = attr.string_list(mandatory = False),
        typesupports = attr.string_list(mandatory = False),
        package = attr.string(mandatory = True),
        interfaces = attr.label_list(
            mandatory = True, allow_files = True
        ),
        includes = attr.label_list(mandatory = False),
        output_dir = attr.string(mandatory = False),
        _tool = attr.label(
            default = "@REPOSITORY_ROOT@:rosidl",
            executable = True, cfg = "exec"
        )
    ),
    implementation = _rosidl_generate_genrule_impl,
    output_to_genfiles = True,
)

def _rosidl_translate_genrule_impl(ctx):
    args = ctx.actions.args()
    args.add("translate")
    output_path = "{}/{}".format(
        ctx.var["GENDIR"],
        ctx.label.package)
    if ctx.attr.output_dir:
        output_path = "{}/{}".format(
            output_path, ctx.attr.output_dir)
    args.add("--output-path", output_path)
    args.add("--output-format", ctx.attr.output_format)
    if ctx.attr.input_format:
        args.add("--input-format", ctx.attr.input_format)
    args.add_all(ctx.files.includes, map_each=_as_include_flag, uniquify=True)
    args.add(ctx.attr.package)
    args.add_all(ctx.files.interfaces, map_each=_as_idl_tuple)
    inputs, input_manifests = ctx.resolve_tools(tools = [ctx.attr._tool])
    inputs = inputs.to_list() + ctx.files.interfaces + ctx.files.includes
    ctx.actions.run(
        executable = ctx.executable._tool,
        arguments = [args], inputs = inputs,
        input_manifests = input_manifests,
        outputs = ctx.outputs.translated_interfaces
    )

rosidl_translate_genrule = rule(
    attrs = dict(
        translated_interfaces = attr.output_list(mandatory = True),
        output_format = attr.string(mandatory = True),
        input_format = attr.string(mandatory = False),
        package = attr.string(mandatory = True),
        interfaces = attr.label_list(
            mandatory = True, allow_files = True
        ),
        includes = attr.label_list(mandatory = False),
        output_dir = attr.string(mandatory = False),
        _tool = attr.label(
            default = "@REPOSITORY_ROOT@:rosidl",
            executable = True, cfg = "exec"
        )
    ),
    implementation = _rosidl_translate_genrule_impl,
    output_to_genfiles = True,
)

def _extract_interface_parts(path):
    parent, _, base = path.rpartition("/")
    basename, _, ext = base.rpartition(".")
    basename = basename[0].lower() + "".join([
        "_" + char.lower() if char.isupper() else char
        for char in basename[1:].elems()
    ])
    return parent, basename

def rosidl_definitions_filegroup(name, group, interfaces, includes, **kwargs):
    translated_interfaces = []
    for ifc in interfaces:
        base, _, ext = ifc.rpartition(".")
        translated_interfaces.append(base + ".idl")
    rosidl_translate_genrule(
        name = name,
        output_format = "idl",
        translated_interfaces = translated_interfaces,
        package = group,
        interfaces = interfaces,
        includes = includes,
        **kwargs
    )

def _generated_source_paths(group, kind):
    base = "{}/{}".format(group, kind)
    include = "{}/{}".format(native.package_name(), base)
    root = "{}/{}".format(base, group)
    return base, root

def rosidl_c_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "c")

    generated_c_sources = []
    visibility_header = "msg/rosidl_generator_c__visibility_control.h"
    generated_c_headers = ["{}/{}".format(root, visibility_header)]
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_c_headers.append("{}/{}/{}.h".format(root, parent, basename))
        generated_c_headers.append("{}/{}/detail/{}__functions.h".format(root, parent, basename))
        generated_c_headers.append("{}/{}/detail/{}__struct.h".format(root, parent, basename))
        generated_c_headers.append("{}/{}/detail/{}__type_support.h".format(root, parent, basename))
        generated_c_sources.append("{}/{}/detail/{}__functions.c".format(root, parent, basename))
    generated_sources = generated_c_sources + generated_c_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        types = ["c"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    deps = deps + [
        "@REPOSITORY_ROOT@:rcutils_cc",
        "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
        "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
    ]

    native.cc_library(
        name = name,
        srcs = generated_c_sources,
        hdrs = generated_c_headers,
        includes = [include],
        deps = deps,
        **kwargs
    )

def rosidl_cc_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "cpp")

    generated_cc_headers = []
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_cc_headers.append("{}/{}/{}.hpp".format(root, parent, basename))
        generated_cc_headers.append("{}/{}/detail/{}__builder.hpp".format(root, parent, basename))
        generated_cc_headers.append("{}/{}/detail/{}__struct.hpp".format(root, parent, basename))
        generated_cc_headers.append("{}/{}/detail/{}__traits.hpp".format(root, parent, basename))

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_cc_headers,
        types = ["cpp"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_library(
        name = name,
        hdrs = generated_cc_headers,
        includes = [include],
        deps = deps + [
            "@REPOSITORY_ROOT@:rosidl_runtime_cpp_cc"
        ],
        **kwargs
    )

def _c_extension(name):
    return name + "_c_extension" + PYTHON_EXTENSION_SUFFIX

def _c_extension_label(name):
    return ":" + _c_extension(name)

def _c_typesupport_extension(package, name):
    return "{}__{}".format(package, name) + PYTHON_EXTENSION_SUFFIX

def _c_typesupport_extension_label(package, name):
    return ":" + _c_typesupport_extension(package, name)

def rosidl_py_library(
    name, group, interfaces, typesupports,
    includes = [], c_deps = [], py_deps = [],
    **kwargs
):
    include, root = _generated_source_paths(group, "py")

    generated_c_sources = []
    generated_py_sources = []
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        py_source = "{}/{}/__init__.py".format(root, parent)
        if py_source not in generated_py_sources:
            generated_py_sources.append(py_source)
        generated_py_sources.append(
            "{}/{}/_{}.py".format(root, parent, basename))
        generated_c_sources.append(
            "{}/{}/_{}_s.c".format(root, parent, basename))
    generated_sources = generated_c_sources + generated_py_sources

    generated_c_sources_per_typesupport = {}
    for typesupport_name in typesupports:
        generated_c_sources_per_typesupport[typesupport_name] = [
            "{}/_{}_s.ep.{}.c".format(root, group, typesupport_name)]
        generated_sources += generated_c_sources_per_typesupport[typesupport_name]

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        types = [
            "py[typesupport_implementations:[{}]]"
            .format(",".join(typesupports))
        ],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = _c_extension(name),
        srcs = generated_c_sources,
        linkshared = 1,
        deps = c_deps + [
            "@python_dev//:libs",
        ],
        **kwargs
    )

    c_typesupport_extension_deps = c_deps + [
        _c_extension_label(name),
        "@REPOSITORY_ROOT@:rosidl_generator_py_cc",
        "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
        "@REPOSITORY_ROOT@:rosidl_typesupport_c_cc",
        "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
        "@python_dev//:libs",
    ]
    py_data = [_c_extension_label(name)]
    for typesupport_name, typesupport_library in typesupports.items():
        deps = list(c_typesupport_extension_deps)
        if typesupport_library not in deps:
            deps.append(typesupport_library)
        native.cc_binary(
            name = _c_typesupport_extension(group, typesupport_name),
            srcs = generated_c_sources_per_typesupport[typesupport_name],
            deps = deps, linkshared = 1, **kwargs
        )
        py_data.append(_c_typesupport_extension_label(group, typesupport_name))

    native.py_library(
        name = name,
        srcs = generated_py_sources,
        data = py_data,
        deps = py_deps,
        **kwargs
    )

def rosidl_typesupport_fastrtps_cc_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/fastrtps_cpp")

    generated_cc_sources = []
    visibility_header = "msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
    generated_cc_headers = ["{}/{}".format(root, visibility_header)]
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        template = "{}/{}/detail/dds_fastrtps/{}__type_support.cpp"
        generated_cc_sources.append(template.format(root, parent, basename))
        template = "{}/{}/detail/{}__rosidl_typesupport_fastrtps_cpp.hpp"
        generated_cc_headers.append(template.format(root, parent, basename))
    generated_sources = generated_cc_sources + generated_cc_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        typesupports = ["fastrtps_cpp"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_cc_sources + generated_cc_headers,
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:fastcdr_cc",
            "@REPOSITORY_ROOT@:rmw_cc",
            "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_fastrtps_cpp_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
        ],
        **kwargs
    )

def rosidl_typesupport_fastrtps_c_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/fastrtps_c")

    generated_c_sources = []
    visibility_header = "msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
    generated_c_headers = ["{}/{}".format(root, visibility_header)]
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        template = "{}/{}/detail/{}__type_support_c.cpp"
        generated_c_sources.append(template.format(root, parent, basename))
        template = "{}/{}/detail/{}__rosidl_typesupport_fastrtps_c.h"
        generated_c_headers.append(template.format(root, parent, basename))
    generated_sources = generated_c_sources + generated_c_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        typesupports = ["fastrtps_c"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_c_sources + generated_c_headers,
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:fastcdr_cc",
            "@REPOSITORY_ROOT@:rmw_cc",
            "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_fastrtps_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_fastrtps_cpp_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
        ],
        **kwargs
    )

def rosidl_typesupport_introspection_c_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/introspection_c")

    generated_c_sources = []
    visibility_header = "msg/rosidl_typesupport_introspection_c__visibility_control.h"
    generated_c_headers = ["{}/{}".format(root, visibility_header)]
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_c_sources.append(
            "{}/{}/detail/{}__type_support.c".format(root, parent, basename))
        template = "{}/{}/detail/{}__rosidl_typesupport_introspection_c.h"
        generated_c_headers.append(template.format(root, parent, basename))
    generated_sources = generated_c_sources + generated_c_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        typesupports = ["introspection_c"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_c_sources + generated_c_headers,
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:rosidl_typesupport_introspection_c_cc",
        ],
        **kwargs
    )

def rosidl_typesupport_introspection_cc_library(
    name, group, interfaces, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/introspection_cpp")

    generated_cc_sources = []
    generated_cc_headers = []
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_cc_sources.append(
            "{}/{}/detail/{}__type_support.cpp".format(root, parent, basename))
        template = "{}/{}/detail/{}__rosidl_typesupport_introspection_cpp.hpp"
        generated_cc_headers.append(template.format(root, parent, basename))
    generated_sources = generated_cc_sources + generated_cc_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        typesupports = ["introspection_cpp"],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_cc_sources + generated_cc_headers,
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_introspection_cpp_cc",
        ],
        **kwargs
    )

def rosidl_typesupport_c_library(
    name, group, interfaces, typesupports, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/c")

    generated_c_sources = []
    visibility_header = "msg/rosidl_typesupport_c__visibility_control.h"
    generated_c_headers = ["{}/{}".format(root, visibility_header)]
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_c_sources.append(
            "{}/{}/{}__type_support.cpp".format(root, parent, basename))
    generated_sources = generated_c_sources + generated_c_headers

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_sources,
        typesupports = [
            "c[typesupport_implementations:[{}]]"
            .format(",".join(typesupports))
        ],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_c_sources + generated_c_headers + [
            ts for ts in typesupports.values()],
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_c_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
        ],
        **kwargs
    )

def rosidl_typesupport_cc_library(
    name, group, interfaces, typesupports, includes = [], deps = [], **kwargs
):
    include, root = _generated_source_paths(group, "typesupport/cpp")

    generated_cc_sources = []
    for ifc in interfaces:
        parent, basename = _extract_interface_parts(ifc)
        generated_cc_sources.append(
            "{}/{}/{}__type_support.cpp".format(root, parent, basename))

    rosidl_generate_genrule(
        name = name + "_gen",
        generated_sources = generated_cc_sources,
        typesupports = [
            "cpp[typesupport_implementations:[{}]]"
            .format(",".join(typesupports))
        ],
        package = group,
        interfaces = interfaces,
        includes = includes,
        output_dir = root,
        **kwargs
    )

    native.cc_binary(
        name = name,
        srcs = generated_cc_sources + [
            ts for ts in typesupports.values()],
        includes = [include],
        linkshared = 1,
        deps = deps + [
            "@REPOSITORY_ROOT@:rosidl_runtime_c_cc",
            "@REPOSITORY_ROOT@:rosidl_runtime_cpp_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_cpp_cc",
            "@REPOSITORY_ROOT@:rosidl_typesupport_interface_cc",
        ],
        **kwargs
    )

def defs(name):
    return name + "_defs"

def cc(name):
    return name + "_cc"

def cc_label(name):
    return ":" + cc(name)

def cc_types(name):
    return name + "__rosidl_cpp"

def cc_types_label(name):
    return ":" + cc_types(name)

def typesupport_cc(name):
    return name + "__rosidl_typesupport_cpp"

def typesupport_cc_label(name):
    return ":" + typesupport_cc(name)

def typesupport_introspection_cc(name):
    return name + "__rosidl_typesupport_introspection_cpp"

def typesupport_introspection_cc_label(name):
    return ":" + typesupport_introspection_cc(name)

def typesupport_fastrtps_cc(name):
    return name + "__rosidl_typesupport_fastrtps_cpp"

def typesupport_fastrtps_cc_label(name):
    return ":" + typesupport_fastrtps_cc(name)

def rosidl_cc_support(name, interfaces, deps, **kwargs):
    rosidl_cc_library(
        name = cc_types(name),
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        deps = [cc(dep) for dep in deps],
        **kwargs
    )

    typesupports = {}

    if "rosidl_typesupport_introspection_cpp" in AVAILABLE_TYPESUPPORTS:
        rosidl_typesupport_introspection_cc_library(
            name = typesupport_introspection_cc(name),
            group = name, interfaces = interfaces,
            includes = [defs(dep) for dep in deps],
            deps = [cc_types_label(name)] + [cc(dep) for dep in deps],
            **kwargs
        )
        typesupports["rosidl_typesupport_introspection_cpp"] = \
            typesupport_introspection_cc_label(name)

    if "rosidl_typesupport_fastrtps_cpp" in AVAILABLE_TYPESUPPORTS:
        rosidl_typesupport_fastrtps_cc_library(
            name = typesupport_fastrtps_cc(name),
            group = name, interfaces = interfaces,
            includes = [defs(dep) for dep in deps],
            deps = [cc_types_label(name)] + [cc(dep) for dep in deps],
            **kwargs
        )
        typesupports["rosidl_typesupport_fastrtps_cpp"] =  \
            typesupport_fastrtps_cc_label(name)

    rosidl_typesupport_cc_library(
        name = typesupport_cc(name),
        typesupports = typesupports,
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        deps = [cc_types_label(name)] + [cc(dep) for dep in deps],
        **kwargs
    )

    native.cc_library(
        name = cc(name),
        srcs = [
            typesupport_cc_label(name),
        ] + typesupports.values(),
        deps = [cc_types_label(name)],
        linkstatic = 1,
        **kwargs
    )

def c(name):
    return name + "_c"

def c_label(name):
    return ":" + c(name)

def c_types(name):
    return name + "__rosidl_c"

def c_types_label(name):
    return ":" + c_types(name)

def py(name):
    return name + "_py"

def py_label(name):
    return ":" + py(name)

def typesupport_c(name):
    return name + "__rosidl_typesupport_c"

def typesupport_c_label(name):
    return ":" + typesupport_c(name)

def typesupport_introspection_c(name):
    return name + "__rosidl_typesupport_introspection_c"

def typesupport_introspection_c_label(name):
    return ":" + typesupport_introspection_c(name)

def typesupport_fastrtps_c(name):
    return name + "__rosidl_typesupport_fastrtps_c"

def typesupport_fastrtps_c_label(name):
    return ":" + typesupport_fastrtps_c(name)

def rosidl_py_support(name, interfaces, deps, **kwargs):
    rosidl_c_library(
        name = c_types(name),
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        deps = [c(dep) for dep in deps],
        **kwargs
    )

    typesupports = {}

    if "rosidl_typesupport_introspection_c" in AVAILABLE_TYPESUPPORTS:
        rosidl_typesupport_introspection_c_library(
            name = typesupport_introspection_c(name),
            group = name, interfaces = interfaces,
            includes = [defs(dep) for dep in deps],
            deps = [c_types_label(name)] + [c(dep) for dep in deps],
            **kwargs
        )
        typesupports["rosidl_typesupport_introspection_c"] = \
            typesupport_introspection_c_label(name)

    if "rosidl_typesupport_fastrtps_c" in AVAILABLE_TYPESUPPORTS:
        rosidl_typesupport_fastrtps_c_library(
            name = name + "_typesupport_fastrtps_c",
            group = name, interfaces = interfaces,
            includes = [defs(dep) for dep in deps],
            deps = [c_types_label(name)] + [c(dep) for dep in deps],
            **kwargs
        )
        typesupports["rosidl_typesupport_fastrtps_c"] = \
            typesupport_fastrtps_c_label(name)

    rosidl_typesupport_c_library(
        name = typesupport_c(name),
        typesupports = typesupports,
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        deps = [c_types_label(name)] + [c(dep) for dep in deps],
        **kwargs
    )
    typesupports["rosidl_typesupport_c"] = typesupport_c_label(name)

    # Use C typesupport library as entrypoint
    native.cc_library(
        name = c(name),
        srcs = typesupports.values(),
        deps = [c_types_label(name)],
        linkstatic = 1,
        **kwargs
    )

    rosidl_py_library(
        name = py(name),
        typesupports = typesupports,
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        py_deps = [py(dep) for dep in deps],
        c_deps = [c(dep) for dep in deps] + [
            c_types_label(name), typesupports["rosidl_typesupport_c"]
        ],
        **kwargs
    )

def rosidl_interfaces_group(name, interfaces, deps, **kwargs):
    rosidl_definitions_filegroup(
        name = defs(name),
        group = name, interfaces = interfaces,
        includes = [defs(dep) for dep in deps],
        **kwargs
    )

    rosidl_cc_support(name, interfaces, deps, **kwargs)

    rosidl_py_support(name, interfaces, deps, **kwargs)
