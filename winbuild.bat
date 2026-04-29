@echo on

rem - this is  a sample build script for building gui_tool MSI file under windows, it assumes the following:
rem - you already have a python.org python installed (at least 3.10) tested on 3.10.2
rem - you have a git checkout of [the correct] gui_tool [release] here

rem - how to use:
rem - step 1 - edit the script to change the PATH below to include your python 3.10 install directory
rem - step 2 - open a command prompt
rem - step 3 - run winbuild.bat in the gui_tool directory

rem NOTE: you need visual studio installed, with the C++ build tools

SET PATH=f:\WinPython;%PATH%

python --version

rem Pin cx_Freeze < 7 (newer versions hit a bytecode-scanner IndexError on
rem this project) and setuptools < 81 (>= 81 removed pkg_resources, which
rem setup.py and pip_sizes.py still import).
python -m pip install -U "cx_Freeze<7" "setuptools<81"
python -m pip install -U pymavlink
python -m pip install -U pywin32
python -m pip install -U python-can
rem Install the bundled pydronecan submodule first so 'dronecan' is
rem registered as a distribution (bdist_msi calls pkg_resources.require).
rem --no-build-isolation forces pip to use the venv's pinned setuptools
rem (instead of fetching the latest, which has removed pkg_resources).
if not exist ".\pydronecan\setup.py" if not exist ".\pydronecan\pyproject.toml" (
    echo.
    echo ERROR: Missing pydronecan submodule or package metadata.
    echo ERROR: Expected .\pydronecan\setup.py or .\pydronecan\pyproject.toml
    echo ERROR: Please run: git submodule update --init --recursive
    echo.
    exit /b 1
)
python -m pip install --no-build-isolation -U .\pydronecan
python -m pip install --no-build-isolation -U .

rem show pip sizes for debug
python pip_sizes.py

rem  make the .msi
python setup.py install
python setup.py bdist_msi

rem find the binary in 'dist' folder
dir dist
