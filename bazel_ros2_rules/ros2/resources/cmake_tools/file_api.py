import json
from pathlib import Path
import subprocess
from typing import List


class Target:
    """Represents a CMake target like a shared library or executable."""

    __slots__ = (
        'name',
        'target_type',
        'compile_flags',
        'defines',
        'link_flags',
        'link_libraries',
        'link_search_paths',
        'includes',
        'system_includes',
    )

    name: str
    target_type: str
    compile_flags: List[str]
    defines: List[str]
    link_flags: List[str]
    link_libraries: List[str]
    link_search_paths: List[str]
    includes: List[str]
    system_includes: List[str]

    def __init__(self, target_file: Path):
        assert target_file.is_file()
        self.name = None
        self.target_type = None
        self.compile_flags = []
        self.defines = []
        self.link_flags = []
        self.link_libraries = []
        self.link_search_paths = []
        self.includes = []
        self.system_includes = []

        target_data = json.loads(target_file.read_text())
        self.name = target_data['name']
        self.target_type = target_data['type']
        if 'compileGroups' in target_data:
            assert len(target_data['compileGroups']) == 1
            compile_group = target_data['compileGroups'][0]

            for fragment in compile_group['compileCommandFragments']:
                self.compile_flags.append(fragment['fragment'])

            for define in compile_group['defines']:
                self.defines.append(define['define'])

            if 'includes' in compile_group:
                for include in compile_group['includes']:
                    if 'isSystem' in include and include['isSystem']:
                        self.system_includes.append(include['path'])
                    else:
                        self.includes.append(include['path'])

        if 'link' in target_data:
            link = target_data['link']
            assert 'CXX' == link['language']

            for fragment in link['commandFragments']:
                if fragment['role'] == 'flags':
                    self.link_flags.append(fragment['fragment'])
                elif fragment['role'] == 'libraries':
                    self.link_libraries.append(fragment['fragment'])
                elif fragment['role'] == 'libraryPath':
                    self.link_search_paths.append(fragment['fragment'])
                elif fragment['role'] == 'frameworkPath':
                    pass
                else:
                    raise RuntimeError(f'Unknown fragment role {fragment}')


class CodeModel:
    """Represents CMake's codemodel v2 from the file API."""

    __slots__ = (
        'targets',
    )

    targets: List[Target]

    def __init__(self, codemodel_path: Path):
        assert codemodel_path.is_file()

        self.targets = []

        codemodel = json.loads(codemodel_path.read_text())

        assert len(codemodel['configurations']) == 1

        cfg = codemodel['configurations'][0]
        for target in cfg['targets']:
            target_file = codemodel_path.parent.joinpath(target['jsonFile'])
            self.targets.append(Target(target_file))


def get_cmake_codemodel(project_path: Path, build_path: Path) -> CodeModel:
    """
    Use the cmake-file-api to get information about the code model of a CMake
    project.

    :param project_path: A path to a CMake project to analyze.
    :param build_path: A path to a build directory for the project to analyze.
    """
    if not build_path.is_dir():
        raise ValueError('The build_path directory must exist')
    if not project_path.joinpath('CMakeLists.txt').is_file():
        raise ValueError('The project_path must contain a CMakeLists.txt')

    api_path = build_path.joinpath('.cmake/api/v1')
    query_path = api_path.joinpath('query')
    query_path.mkdir(parents=True)

    # Make a shared stateless query for the codemodel
    query_path.joinpath('codemodel-v2').touch()

    args = ['cmake', str(project_path)]
    subprocess.run(args, cwd=str(build_path))

    # There should only be one file, but just in case the latest is the
    # largest file in lexicographic order.
    reply_path = api_path.joinpath('reply')
    latest_reply = sorted(reply_path.glob('index-*.json'))[-1]

    reply = json.loads(latest_reply.read_text())

    codemodel_reply_file = None
    for obj in reply['objects']:
        if 'codemodel' == obj['kind']:
            codemodel_reply_file = obj['jsonFile']

    assert codemodel_reply_file
    codemodel_reply_file = reply_path.joinpath(codemodel_reply_file)

    return CodeModel(codemodel_reply_file)
