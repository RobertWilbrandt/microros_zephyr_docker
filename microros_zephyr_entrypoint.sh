#!/bin/bash
set -e

echo "-------------------------"
echo "| microros_zehyr_docker |"
echo "-------------------------"
echo ""

pushd /uros_apps >/dev/null

# Check if apps were mounted correctly
if ls -1qA . | grep -q .
then

  echo "Found custom apps:"
  for app in $(ls -d */ | cut -f1 -d'/'); do
    echo " - $app"
  done
  echo ""

else
  echo "$(tput setaf 1)Could not find any custom apps. Make sure you mapped your apps correctly using the$(tput sgr0)"
    echo "  --volume <absolute_path_to_your_apps_folder>:/uros_apps"
    echo "$(tput setaf 1)argument as described in the README.$(tput sgr0)"
  exit 1
fi

popd >/dev/null

exec "$@"
