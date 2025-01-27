# CS 548: 3D Data Processing
***Spring 2025***  
***Author: Matthew Townsend***  
***Original Author: Dr. Michael J. Reale***  
***SUNY Polytechnic Institute*** 

## Software Dependencies Overview
You will need to install the following manually:
- C++ compilers
- CMake
- Miniconda
- PCL
- Visual Code

The rest of the dependencies will be automatically fetched by CMake.

You will then need to create a miniconda environment.

## C++ Compilers
* ***For Windows:*** Download and install [Visual Studio 2022](https://visualstudio.microsoft.com/)  
  * Under Workloads, add ```Desktop development with C++```
  * Under "Individual Components", search for and add: ```C++ MFC for latest v143 build tools (x86 & x64)```
* ***For Linux:*** Install the GNU g++ compilers: ```sudo apt install g++ make```
* ***For Mac:*** The included Clang g++ compilers should theoretically work.

## CMake
You will need **version 3.24** or higher.

* ***For Windows:***
  * Download the CMake installer from [here](https://cmake.org/download/).
  * If given the option, make sure to **"Add CMake to the system PATH"**.
* ***For Linux:***
  * Install the following dependencies first:
```
sudo apt install libssl-dev
```
  * Download the latest CMake source tar.gz file [here](https://cmake.org/download/).
  * Run the following to compile and install it:
```
tar -xvzf cmake-*.tar.gz 
cd cmake-*/
./bootstrap 
make 
sudo make install
cmake --version
```
* ***For Mac:***
  * Download the latest "macOS 10.13 or later" .dmg from [here](https://cmake.org/download/)
  * Install Cmake and run it
  * Select ```"Tools" -> "How to Install for Command Line Use"```
  * Copy the second option into a terminal: ```sudo "/Applications/CMake.app/Contents/bin/cmake-gui" --install```


## Python (Miniconda) Installation

Miniconda is a smaller version of Anaconda, which is a distribution of Python.  You will need to at least install Miniconda on a local machine (we will discuss how to make a portable version later in these instructions).

### Windows

First, download the latest installer for Windows [here](https://docs.conda.io/projects/miniconda/en/latest/).

Run the installer; I would install it for All Users (especially if your username has spaces in it).  I installed it into ```C:/miniconda3```.

Open "Anaconda Prompt (miniconda3)" **as an administrator**; you should see ```(base)``` to the left of your terminal prompt.

### Linux

First, download the latest version:
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

Next, install it into your home directory (default options are fine):
```
~/miniconda3/bin/conda init bash
```

Close and reopen the terminal; you should see ```(base)``` to the left of your terminal prompt.

### Mac
Open up a terminal.

If you are on an **Intel Mac**:
```
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh
sh Miniconda3-latest-MacOSX-x86_64.sh
```
If you are on an **M1 Mac**:
```
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
sh Miniconda3-latest-MacOSX-arm64.sh
```
Close and reopen the terminal; you should see ```(base)``` to the left of your terminal prompt.

## Python Environment Creation

Create your DATA3D environment:
```
conda create -n DATA3D python=3.11
```

Before installing any packages to the new environment, activate it:
```
conda activate DATA3D
```

```(DATA3D)``` should now be to the left of your terminal prompt.

## Installing Packages

***For Windows:*** Open "Anaconda Prompt (miniconda3)" **as an administrator**.  

***For Linux:*** If conda is installed system-wide, make sure the commands that follow are run using ``sudo``.

Activate your environment:
```
conda activate DATA3D
```

### Updating pip
On Linux in particular, with your environment activated, make sure pip is updated:

```
python -m pip install --upgrade pip
```

### Installing PyTorch
With ```DATA3D``` activated, you will need to install PyTorch for deep learning.  

The latest instructions for installing PyTorch may be found [here](https://pytorch.org/get-started/locally/).

#### CUDA-Enabled PyTorch
If you have an NVIDIA graphics card (or intend to run this environment on a machine with one), you will want to install the CUDA-enabled version of PyTorch.

***NOTE:*** On Linux, a version of Open3D may be installed that is pytorch-enabled, allowing usage of Open3D-ML.  However, accordingly to the documentation, you must install ```torch==2.2.2+cu121``` and ```torchvision==0.17.2+cu121```.  For consistency, we will use these versions for Windows and Mac as well.

The following instructions are for **CUDA version 12.1**.

##### Windows

```
pip3 install torch==2.2.2+cu121 torchvision==0.17.2+cu121 torchaudio --index-url https://download.pytorch.org/whl/cu121
```

##### Linux

```
pip3 install torch==2.2.2+cu121 torchvision==0.17.2+cu121 torchaudio --index-url https://download.pytorch.org/whl/cu121
```

#### CPU/Mac PyTorch
If you do NOT have an NVIDIA card (and/or are on a Mac), you will need to settle for the default installation:

```
pip3 install torch torchvision torchaudio
```

### Verifying PyTorch
To verify Pytorch works correctly:
```
python -c "import torch; x = torch.rand(5, 3); print(x); print(torch.cuda.is_available())"
```
You should see an array printed.  

If you have an NVIDIA card and installed the CUDA version of PyTorch, you should also see the word ```True``` (otherwise, ```False``` is expected).

### Installing Open3D
To install Open3D:
```
pip3 install open3d
```
This should also install Open3D-ML.

### Verifying Open3D
To verify Open3D has been installed correctly:
```
python -c "import open3d as o3d; print(o3d.__version__)"

python -c "import open3d as o3d; mesh = o3d.geometry.TriangleMesh.create_sphere(); mesh.compute_vertex_normals(); o3d.visualization.draw(mesh, raw_mode=True)"
```
First, the version of Open3D installed should print out.  Then, you should see a visualization window pop up.

To verify the command-line applications work:
```
open3d example visualization/draw
```
Similarly, a series of visualization windows should pop up.

### Verifying Open3D-ML
To verify Open3D-ML has been installed correctly:
```
pip3 install tensorboard
python -c "import open3d.ml.torch as ml3d"
```
***Under Linux***, this line should execute without errors.

### Installing Other Python Packages

Then, run the following commands to install the necessary packages for this course:
```
pip3 install tensorboard
pip3 install py3dtiles
pip3 install deepgeo
pip3 install pandas
pip3 install scikit-learn scikit-image 
pip3 install matplotlib 
pip3 install pytest
conda install conda-pack
```

### Python Environment Troubleshooting

* If you need to remove the DATA3D environment (the LOCAL version, not the portable one):
```
conda remove --name DATA3D --all
```

* If you encounter path issues (where conda isn't found once you activate an environment), open an Anaconda prompt as admin and try the following to add conda to your path globally: 
```
conda init
```

## PCL

### Windows

#### Installer for VS 2022
An [installer](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/PCL-1.14.1-AllInOne-msvc2022-win64.exe) exists for Visual Studio 2022, x64 under [Releases](https://github.com/PointCloudLibrary/pcl/releases) on PCL's GitHub page.

Running this installer will install PCL (and OpenNI2) to Program Files, but the installed libraries should only be about 2GB in size.

You will need to ensure that PCL's bin directory (and a few others) are on the path, since your application will now need access to certain .dlls to run.

When running the installer, make sure to select ```Add PCL to the system PATH for all users```.  You may get an error saying that the PATH variable cannot be modified.  If so, manually add the following to your system ```Path``` variable:
```
C:\Program Files\PCL 1.14.1\bin
C:\Program Files\PCL 1.14.1\3rdParty\VTK\bin
C:\Program Files\OpenNI2\Redist
```

#### Compiling from Source
If the installer does not work, we can use **Vcpkg** (a package manager for Windows) to compile PCL from source.

**PLEASE NOTE: the compilation process takes up a lot of time (hours) and space (~50GB to ~100GB).**

1. Open a **Developer Command Prompt for VS 2022** ***as an Administrator***.
2. cd to a folder where you can safely download and compile sources (e.g., ```C:/Code```).
3. Clone the Vcpkg project and run the bootstrap script:
```
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```
1. Compile PCL (THIS WILL TAKE A WHILE):
```
.\vcpkg install pcl[visualization] --triplet x64-windows --host-triplet x64-windows
```
Add --recurse option if you are rebuilding it with new features.

If you need to start over:
```
.\vcpkg remove pcl
```

5. Add a system environment variable:
   - ***Name:*** MY_VCPKG_ROOT
   - ***Value:*** "C:/Code/vcpkg"

### Linux
You should be able to install PCL via the package manager:
```
sudo apt get install libpcl-dev
```

### Mac
1. Install [Homebrew](https://brew.sh/)
2. Run the following: ```brew install pcl```

## Visual Code
I strongly recommend Visual Code as your IDE.
* [Download](https://code.visualstudio.com/) and install Visual Code
  * I suggest enabling the context menu options.
  * ***For Mac:*** Follow [these instructions](https://code.visualstudio.com/docs/setup/mac)
* Install the following extensions:
  * **"C/C++ Extension Pack"** by Microsoft
  * **"Python Extension Pack"** by Don Jayamanne  
  * **"Git Graph"** by mhutchie
  * **"Markdown All in One"** by Yu Zhang

A terminal can always be created/opened with ```View menu -> Terminal```.  However, if you need to restart, click the garbage can icon on the terminal window to destroy it.

***For Windows:*** Change your default terminal from Powershell to Command Prompt:
1. ```View menu -> Command Palette -> "Terminal: Select Default Profile"```
2. Choose ```"Command Prompt"```
3. Close any existing terminals in Visual Code

Once you open a project, to make sure you are using the correct Python interpreter:
1. Close any open terminals with the garbage can icon
2. Open a .py file
3. ```View -> Command Palette -> "Python: Select interpreter"```
4. Choose the one located at under the ```DATA3D``` environment.
5. If the GPU is not being utilized (or you see errors about paths to CUDA libraries not being found), type the following in your terminal to manually activate your environment: ```DATA3D\Scripts\activate.bat``` (you will need whatever path is before ```DATA3D```)

### Portable Visual Code
Go [here](https://code.visualstudio.com/Download) and download the **zip version** of Visual Code for your platform (most likely x64).
Unpack it to your USB drive.  Inside the folder for Visual Code, create a ```data``` folder:

This will cause Visual Code to store extensions and settings locally.

To run Visual Code:

***For Windows:*** double-click on this version of ```Code.exe```.

***For Linux/Mac:*** run ```./code```

## Licenses and Citations
This project relies on the following dependencies:

- [Open3D](https://www.open3d.org), an open-source library for 3D data processing.
  - Open3D is licensed under the **MIT License**.
  - Original paper: [Zhou, Park, and Koltun (2018)](https://arxiv.org/abs/1801.09847)
  - Open3D GitHub: [https://github.com/isl-org/Open3D](https://github.com/isl-org/Open3D)
- [Point Cloud Library (PCL)](https://pointclouds.org) for point cloud processing.
  - PCL is licensed under the **BSD 3-Clause License**.
  - Original paper: [Rusu & Cousins (2011)](https://ieeexplore.ieee.org/document/5980567)
  - PCL GitHub: [https://github.com/PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)
