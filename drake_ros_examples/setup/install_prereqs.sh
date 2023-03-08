$!/bin/bash
set -euo pipefail

cat_without_comments() {
    sed -e 's|#.*||g;' "$(dirname $0)/$1"
}

apt_install () {
  apt-get -q install --no-install-recommends "$@"
}

apt_install $(cat_without_comments packages-ros2.txt)
