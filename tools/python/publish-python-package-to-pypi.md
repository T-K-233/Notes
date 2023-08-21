# Publish Python Package to PyPi

### Prerequisites

```bash
py -m pip install --upgrade pip
py -m pip install --upgrade build
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
│       ├── package_name/
│       │   ├── __init__.py
│       │   └── module.py
│       └── __init__.py
├── LICENSE
├── pyproject.toml
├── README.md
├── setup.py
└── tests/
```

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



