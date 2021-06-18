import glob
import os


def get_packages_with_prefixes(prefixes):
    suffixes = 'Config.cmake', '-config.cmake'
    return {
        os.path.basename(path)[:-len(suffix)]: prefix
        for prefix in prefixes for suffix in suffixes
        for path in glob.glob(
            '{}/*/**/*{}'.format(
                prefix, suffix),
            recursive=True
        )
    }
