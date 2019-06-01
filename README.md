## Set up instructions for WIN 10 64-bit.
### Set up Dependencies.
[Download and install VS 2013](https://my.visualstudio.com/Downloads?q=visual%20studio%202013&wt.mc_id=o~msft~vscom~older-downloads)

[Download CMake 3.14.5 Windows installer](https://github.com/Kitware/CMake/releases/download/v3.14.5/cmake-3.14.5-win64-x64.msi)

[Download Boost 1.65](https://dl.bintray.com/boostorg/release/1.65.0/binaries/boost_1_65_0-msvc-12.0-64.exe)

[Download QHULL and install](https://sourceforge.net/projects/pointclouds/files/dependencies/qhull-6.2.0.1385-vs2010-x64.exe/download)

[Download Eigen and install](https://sourceforge.net/projects/pointclouds/files/dependencies/qhull-6.2.0.1385-vs2010-x64.exe/download)

[Download QT 5 and install](https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5)


[Download and install FLANN](https://sourceforge.net/projects/pointclouds/files/dependencies/flann-1.7.1-vs2010-x64.exe/download)

[Download and install OpenNI](https://sourceforge.net/projects/pointclouds/files/dependencies/OpenNI-Win64-1.5.4-Dev.msi/download)

On VS 2013, install the 'QT Visual Studio Tools' extension via the "Tools>Extensions and Updates" window


[Download VTK 7.1.1 from the source.](https://www.vtk.org/files/release/7.1/VTK-7.1.1.zip)

After Unziping the zip file, open CMAKE-GUI (as administrator)
On CMAKE:
- Put source code path to the path of the unziped VTK code
- On build, create a folder that is easier to access for you, for conviniance is often a good practice to have it in the same path as the source with a folder name like 'bin' or 'build
- Click 'Configure' and select 'Visual Studio 12 2013' as the generator and 'x64' as the optional plataform for generator. Then click finish and wait for the configuration to be done.
- Once done, click on generate, once that finish click on Open project. (open with Visual studio 2013 if asked)
- On Visual Studio 2013:
     - Under the solution explorer, right click on the 'ALL_BUILD' project and click on 'build'
     - Once the build is done, repeat the same under the 'release' configuration.
     - Once both the debug and release builds are over, do the same with the 'INSTALL' project with both debug and release.

## Installing PCL

[Download the source code of PCL 1.8.1](https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.zip)

Like VTK, Unzip the source code and open CMAKE-GUI.
  
On CMake:	

- Put source code path to the path of the unziped VTK code.
- On build, create a folder that is easier to access for you, for conviniance is often a good practice to have it in the same path as the source with a folder name like 'bin' or 'build'
- Click 'Configure' and select 'Visual Studio 12 2013' as the generator and 'x64' as the optional plataform for generator. Then click finish and wait for the configuration to be done.
- Some errors might arise during the configuration, here are some of the more commong onces

       - If CMAKE errors saying that DOXYGEN is missing, install it (http://www.doxygen.nl/download.html) and once doxygen is installed, run the configuration again.

       - If CMAKE errors saying that it does not detect the boost libraries, 
       
       - Go to the entry under "BOOST_LIBRARYDIR" and go paste the path that boost libraries were installed (example: "C:\local\boost_1_65_0\lib64-msvc-12.0")
       
       - Go to the entry under "BOOST_INCLUDEDIR" and go paste the path that boost libraries were installed (example: "C:\local\boost_1_65_0\boost")
- If CMAKE errors saying that it didn't found the version of VTK, go to the corresponding entry and set it with the path where vtk got installed (example: "C:/Program Files (x86)/VTK/lib/cmake/vtk-7.1")

-Once the configuration ends Generate the project and open it in VS 2013.

  On Visual Studio 2013:
  - Under the solution explorer, right click on the 'ALL_BUILD' project and click on 'build'
         - There might be some errors:
                - If there is an access error ("Could not open file") go back to CMAKE and verify that the boost libraries were all detected and that vtk is also set.
                - If there is an syntax error ("ImmediateRenderingOff is not a part of vtkmapper") make sure you have the version 7.1 of VTK as future versions removed said function.
                - If there are syntax warnings they are safe to ignore as most of them only metion lost of pressision in data.
  - Once the build is done, repeat the same under the 'release' configuration.
  - Once both the debug and release builds are over, do the same with the 'INSTALL' project with both debug and release.
				
# Building the Sonar cpp Project:

Once the project is downloaded in the system, open a CMAKE gui in administrator 
- put the source path as "{PROJECT_PATH}" and the destination as "{PROJECT_PATH}/build"
- Set "Visual Studio 12 2013" as the generator and x64 as the opional plataform. After that click finish.
- There might be an message error pop up the first time; if that's the case, just click configure once again, the configuration should complete this time.
- If any more messages pop up, check that the VTK path is set as well as the BOOST paths if asked.
- Once the configuration is complete, generate the project and the open the generated project.
- On Visual Studio 2013: 
       - Go to ALL_BUILD and build it. 
       - Once this is done you should be able to see that an executable called 'detection.exe' in "{PROJECT_PATH}/build/Debug" to test this executable with ease, move the exe file to the root path of the project.

When trying to run 'detection.exe' there will be a chance that the error 
```
The code execution cannot proceed because pcl_common_debug.dll was not found.
```

To fix this, we need to add the installation path of PCL and VTK to our PATH in the enviorment variables.

- To access the Enviorment Variables fast press "ctrl+R" and on the run dialog option copy and paste the path "C:\Windows\System32\systempropertiesadvanced.exe"; this will open the System propeties window.
- Go to Enviorment Variables; on either user variables or system variables select the "Path" variable and click on "edit..."
- In the Edit enviorment variable dialog, click on "new"
- Once in "new" line, we will put the path where PCL got install and contains all the dll's and executables. (should look like "C:\Program Files (x86)\PCL\bin")
- Then we will do the same with VTK (default path should be "C:\Program Files (x86)\VTK\bin")
- Press Ok on both windows to apply the variables. Once done, detection.exe should be able to run.

To test swiftly test detection.exe, drag the executable to the root path of the project, then open a commandline or powershell and type 
> .\detection.exe {Input PCD file} {Output PCD file} {Output Text file}

