set -e
SIMPLEWALKER_ROOT=$(dirname "$(dirname "$(realpath "${BASH_SOURCE[0]}")")")
pushd "$SIMPLEWALKER_ROOT" > /dev/null
uv tool install -q --with "pytest,numpy>=2.0,scipy>1.13" coverage
coverage run -m pytest -q --capture=no
if [[ " $@ " =~ " md " ]]; then
  coverage report --format=markdown
else
  coverage report
fi
if [[ " $@ " =~ " html " ]]; then
  coverage html
fi
popd > /dev/null
