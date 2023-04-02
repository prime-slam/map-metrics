SETLOCAL EnableDelayedExpansion

If "%~1"=="x64" (
    SET plat="x64" || EXIT /B !ERRORLEVEL!
) else (
    SET plat="Win32" || EXIT /B !ERRORLEVEL!
) 

:: Early check for build tools
cmake --version || EXIT /B !ERRORLEVEL!

pushd %~dp0 
cd .. 

mkdir .\build || popd && EXIT /B !ERRORLEVEL!
mkdir .\map_metrics || popd && EXIT /B !ERRORLEVEL!
copy .\__init__.py .\map_metrics\__init__.py || popd && EXIT /B !ERRORLEVEL!


for /D %%P in (C:\hostedtoolcache\windows\Python\3*) do CALL :build %plat% %%P\%~1\python.exe || popd && EXIT /B !ERRORLEVEL!

python -m pip install --user -q build || popd && EXIT /B !ERRORLEVEL!
python -m build --wheel --outdir dist . || popd && EXIT /B !ERRORLEVEL!

popd
EXIT /B 0

:build
cmake -S . -B build -G "Visual Studio 16 2019" -A "%~1" -DPYTHON_EXECUTABLE:FILEPATH=%~2 -DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=%cd%\map_metrics -DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE=%cd%\map_metrics || EXIT /B !ERRORLEVEL!
cmake --build build --config Release || EXIT /B !ERRORLEVEL!
EXIT /B 0
