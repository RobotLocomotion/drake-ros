from typing import Tuple


class CcProperties:

    __slots__ = (
        'link_libraries',
        'include_directories',
        'compile_flags',
        'defines',
        'link_flags',
        'link_directories'
    )

    link_libraries: Tuple[str]
    include_directories: Tuple[str]
    compile_flags: Tuple[str]
    defines: Tuple[str]
    link_flags: Tuple[str]
    link_directories: Tuple[str]

    def __init__(self, **kwargs):
        for name in self.__slots__:
            setattr(self, name, tuple())
        for key, value in kwargs.items():
            if key not in self.__slots__:
                raise TypeError(f'Unexpected keyword argument {key}')
            if not value:
                value = tuple()
            else:
                value = tuple(value)
            setattr(self, key, value)
