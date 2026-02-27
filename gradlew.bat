@rem
@rem Copyright 2015 the original author or authors.
@rem
@rem Licensed under the Apache License, Version 2.0 (the "License");
@rem you may not use this file except in compliance with the License.
@rem You may obtain a copy of the License at
@rem
@rem      https://www.apache.org/licenses/LICENSE-2.0
@rem
@rem Unless required by applicable law or agreed to in writing, software
@rem distributed under the License is distributed on an "AS IS" BASIS,
@rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@rem See the License for the specific language governing permissions and
@rem limitations under the License.
@rem

@if "%DEBUG%"=="" @echo off
@rem ##########################################################################
@rem
@rem  Gradle startup script for Windows
@rem
@rem ##########################################################################

@rem Set local scope for the variables with windows NT shell
if "%OS%"=="Windows_NT" setlocal

set DIRNAME=%~dp0
if "%DIRNAME%"=="" set DIRNAME=.
@rem This is normally unused
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%

@rem Resolve any "." and ".." in APP_HOME to make it shorter.
for %%i in ("%APP_HOME%") do set APP_HOME=%%~fi

@rem Add default JVM options here. You can also use JAVA_OPTS and GRADLE_OPTS to pass JVM options to this script.
set DEFAULT_JVM_OPTS="-Xmx64m" "-Xms64m"

@rem Find java.exe
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if %ERRORLEVEL% equ 0 goto checkJavaCompatibility

echo.
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH.
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:findJavaFromJavaHome
set "JAVA_HOME=%JAVA_HOME:"=%"
set "JAVA_EXE=%JAVA_HOME%\bin\java.exe"

if exist "%JAVA_EXE%" goto checkJavaCompatibility

echo.
echo ERROR: JAVA_HOME is set to an invalid directory: %JAVA_HOME%
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:checkJavaCompatibility
call :detectJavaVersion
if %JAVA_MAJOR% geq 25 goto tryJava21
goto execute

:tryJava21
if not defined JAVA21_HOME goto tryJava17
set "JAVA_HOME=%JAVA21_HOME:"=%"
set "JAVA_EXE=%JAVA_HOME%\bin\java.exe"
if not exist "%JAVA_EXE%" goto tryJava17
call :detectJavaVersion
if %JAVA_MAJOR% geq 25 goto tryJava17
echo Gradle 8.5 detected Java %JAVA_VERSION%; using compatible JAVA_HOME=%JAVA_HOME%.
goto execute

:tryJava17
if not defined JAVA17_HOME goto javaTooNew
set "JAVA_HOME=%JAVA17_HOME:"=%"
set "JAVA_EXE=%JAVA_HOME%\bin\java.exe"
if not exist "%JAVA_EXE%" goto javaTooNew
call :detectJavaVersion
if %JAVA_MAJOR% geq 25 goto javaTooNew
echo Gradle 8.5 detected Java %JAVA_VERSION%; using compatible JAVA_HOME=%JAVA_HOME%.
goto execute

:javaTooNew
echo.
echo ERROR: Gradle 8.5 cannot run on Java %JAVA_VERSION%.
echo.
echo Install Java 21 or 17 and set JAVA_HOME (or JAVA21_HOME/JAVA17_HOME), then retry.
echo.
goto fail

:detectJavaVersion
set JAVA_VERSION=
set JAVA_MAJOR=0
for /f "tokens=3" %%v in ('"%JAVA_EXE%" -version 2^>^&1 ^| findstr /i " version "') do (
  set JAVA_VERSION=%%~v
  goto detectJavaVersionDone
)
:detectJavaVersionDone
set JAVA_VERSION=%JAVA_VERSION:"=%
for /f "tokens=1 delims=." %%m in ("%JAVA_VERSION%") do set JAVA_MAJOR=%%~m
if "%JAVA_MAJOR%"=="1" for /f "tokens=2 delims=." %%m in ("%JAVA_VERSION%") do set JAVA_MAJOR=%%~m
for /f "delims=0123456789" %%m in ("%JAVA_MAJOR%") do set JAVA_MAJOR=0
if "%JAVA_MAJOR%"=="" set JAVA_MAJOR=0
exit /b 0

:execute
@rem Setup the command line

set CLASSPATH=%APP_HOME%\gradle\wrapper\gradle-wrapper.jar


@rem Execute Gradle
"%JAVA_EXE%" %DEFAULT_JVM_OPTS% %JAVA_OPTS% %GRADLE_OPTS% "-Dorg.gradle.appname=%APP_BASE_NAME%" -classpath "%CLASSPATH%" org.gradle.wrapper.GradleWrapperMain %*

:end
@rem End local scope for the variables with windows NT shell
if %ERRORLEVEL% equ 0 goto mainEnd

:fail
rem Set variable GRADLE_EXIT_CONSOLE if you need the _script_ return code instead of
rem the _cmd.exe /c_ return code!
set EXIT_CODE=%ERRORLEVEL%
if %EXIT_CODE% equ 0 set EXIT_CODE=1
if not ""=="%GRADLE_EXIT_CONSOLE%" exit %EXIT_CODE%
exit /b %EXIT_CODE%

:mainEnd
if "%OS%"=="Windows_NT" endlocal

:omega
