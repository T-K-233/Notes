# Publish Python Package to PyPi

### Prerequisites

```bash
py -m pip install --upgrade pip
py -m pip install --upgrade build
py -m pip install --upgrade twine
```

### Project Directory Structure

```
package_name/
├── dist/
│   ├── *.whl
│   └── *.tar.gz
├── examples/
├── src/
│   └── cc/
│       ├── package_name_a/
│       │   ├── __init__.py
│       │   └── module.py
│       └── package_name_b/
│           ├── __init__.py
│           └── module.py
├── LICENSE
├── pyproject.toml
├── README.md
├── setup.py
└── tests/
```

See here for namespaced package structure:

{% embed url="https://packaging.python.org/en/latest/guides/packaging-namespace-packages/" %}



### Commands

Build

```bash
py -m build
```

Install

```bash
pip install .\dist\package_name-ver.si.on-py3-none-any.whl
```

Upload

```bash
py -m twine upload .\dist\*
```
