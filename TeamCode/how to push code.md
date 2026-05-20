open terminal in the bottom right
run "adb connect 192.168.43.1:5555"

if that doesnt work then run "& "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe" connect 192.168.43.1:5555"

to avoid doing the command above, you can do the steps below for a permanent solution:
Press the Windows Key, type Environment Variables, and press Enter
Click the Environment Variables... button at the bottom right.Under User variables, click on Path, then click Edit...
Click New and paste this path: %LOCALAPPDATA%\Android\Sdk\platform-tools