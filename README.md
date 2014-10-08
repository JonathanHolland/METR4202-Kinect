METR4202-Kinect
===============
Jonathan Holland &amp; Alistair Roe

METR4202 Project 2 - Image recognition with the Kinect RGBA &amp; Depth (IR) sensors of takeaway coffee cups and glasses.
For the Full Specifications of The University of Queenslands METR4202 Advanced Robotics course project 2, please see projectSpec.pdf.

Steps to install/run the VS C++ Kinect sample in Windows
-------------------------------------------------
-------------------------------------------------
- Ensure Visual Studio is installed (2013 would be best)
- Install the Kinect for windows SDK 
- Download and install OpenCV for Windows
  
- Extract the 4202p2.zip file of the sample and then open Visual Studio to import this project template.

- add *OpenCVFolder\include* and *KinectSDKFolder\inc* to the C\C++->General->Additional Include Directies Tab under **Project Properties**
  
- *Add OpenCVFolder\x64\vc12\bin*, *OpenCVFolder\x64\vc12\lib* and *KinectSDKFolder\lib\amd64* to the Linker->General->Additional Library Directories Tab
  
- Add the following (omit the trailing letter d for the Release version if desired) to the Linker->Input->Additional Dependencies tab
    - *Kinect10.lib*
    - *opencv_core249d.lib*
    - *opencv_highgui249d.lib*
    - *opencv_features2d249d.lib*
    - *opencv_calib3d249d.lib*
    - *opencv_flann249d.lib*
    - *opencv_nonfree249d.lib*
    - *opencv_imgproc249d.lib*

Done! You are now ready to run and edit the sample!
(Note: The zipped sample is natively of a x64 Architecture)

--------------------------------------------------------
Alternative Setups for Image Recognition with the Kinect
--------------------------------------------------------
--------------------------------------------------------

There are many alternatives to setting up Image Recognition with the Kinect. Some of these differences that are still compatible with Windows and Visual Studio but require some custom compiling of OpenCV include 
- OpenNI (http://structure.io/openni) and 
- LibFreeNect (https://github.com/OpenKinect/libfreenect).

Note that while these libraries contain some functions to make everything easier, this sample is not tailored to either and you *may* lose some freedom of what you can achieve. This is mentioned with the knowledge that LibFreeNect's driver connection is incompatible with the Kinect for windows SDK and that LibFreeNect and OpenNI can't be used simultaneously.

**This was made using Dillinger, a 100% Open Source Online Markdown Editor!**

**Free Software, Hell Yeah!**
