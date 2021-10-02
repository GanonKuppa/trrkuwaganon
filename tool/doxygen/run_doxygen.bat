echo ===== run doxygen =====
rmdir /s /q html/html > NUL 2>&1 & if ERRORLEVEL 1 cmd /c exit 0
doxygen DoxyFile
start html/html/index.html
