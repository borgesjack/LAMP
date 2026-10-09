FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
ARG RAVE_COMMIT=ec22ecfaf006688cbc5ee0fdd8fa05d2c5676d37
ARG OSG_COMMIT=1f89e6eb1087add6cd9c743ab07a5bce53b2f480
ENV QT_X11_NO_MITSHM=1
ENV OPENRAVE_PLUGINS=/usr/local/lib/openrave0.149-plugins
ENV LC_ALL=C.UTF-8
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:${PYTHONPATH}

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Enable universe and multiverse repos (ubuntu:22.04 only has main by default)
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y universe && \
    add-apt-repository -y multiverse && \
    apt-get update

# System dependencies - Python 3.10 (system default on 22.04)
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    coreutils \
    nano \
    vim \
    tmux \
    ipython3 \
    minizip \
    wget

# C++ build dependencies
RUN apt-get install -q -y --no-install-recommends \
    libassimp-dev \
    libavcodec-dev \
    libavformat-dev \
    libboost-all-dev \
    libboost-date-time-dev \
    libboost-python-dev \
    libbullet-dev \
    libglew-dev \
    libgsm1-dev \
    liblapack-dev \
    liblog4cxx-dev \
    libmpfr-dev \
    libode-dev \
    libogg-dev \
    libpcrecpp0v5 \
    libpcre3-dev \
    libqhull-dev \
    libswscale-dev \
    libvorbis-dev \
    libx264-dev \
    libxml2-dev \
    libxvidcore-dev \
    libbz2-dev \
    libtinyxml-dev \
    libmpfi-dev \
    libfreetype6-dev \
    libflann-dev \
    libeigen3-dev \
    libann-dev \
    ann-tools \
    libccd-dev \
    octomap-tools \
    libminizip-dev \
    libcollada-dom2.4-dp-dev \
    liblapacke-dev \
    libgsl-dev \
    libcairo2-dev \
    libpoppler-glib-dev \
    libsdl2-dev \
    libtiff5-dev \
    libxrandr-dev \
    libyaml-cpp-dev \
    pybind11-dev \
    rapidjson-dev \
    qtbase5-dev \
    libqt5opengl5-dev

# Install collada-dom
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/rdiankov/collada-dom.git && \
    cd collada-dom && mkdir build && cd build && \
    cmake .. && \
    make -j $(nproc) && \
    make install

# Install OpenSceneGraph
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/openscenegraph/OpenSceneGraph.git && \
    cd OpenSceneGraph && git reset --hard ${OSG_COMMIT} && \
    mkdir build && cd build && \
    cmake -DDESIRED_QT_VERSION=5 .. && \
    make -j $(nproc) && make install && make install_ld_conf

# Install FCL 0.5.0
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/flexible-collision-library/fcl && \
    cd fcl && git reset --hard 0.5.0 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j $(nproc) && \
    make install

# Install newer RapidJSON (system 1.1.0 lacks copyConstStrings needed by OpenRAVE master)
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/Tencent/rapidjson.git && \
    cd rapidjson && mkdir build && cd build && \
    cmake -DRAPIDJSON_BUILD_DOC=OFF -DRAPIDJSON_BUILD_EXAMPLES=OFF -DRAPIDJSON_BUILD_TESTS=OFF .. && \
    make install

# Install OpenRAVE with Python 3.10 pybind11 bindings
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install sympy "numpy<2" "pybind11>=2.9.2,<3"
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/rdiankov/openrave.git && \
    cd openrave && git reset --hard ${RAVE_COMMIT} && \
    mkdir build && cd build && \
    cmake -DODE_USE_MULTITHREAD=ON \
          -DCMAKE_CXX_STANDARD=17 \
          -DCMAKE_CXX_FLAGS="-Wno-error=narrowing -fpermissive" \
          -DOPT_PYTHON=OFF \
          -DOPT_PYTHON3=ON \
          -DOPT_FCL_COLLISION=OFF \
          -DOPT_MSGPACK=OFF \
          -DOPT_ENCRYPTION=OFF \
          -DBoost_NO_BOOST_CMAKE=TRUE \
          -DBoost_NO_SYSTEM_PATHS=FALSE \
          -DOSG_DIR=/usr/local \
          -DCMAKE_LIBRARY_PATH=/usr/local/lib64 \
          -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir) .. && \
    make -j $(nproc) && \
    make install

# Configure locale and shared library cache
RUN apt-get update && apt-get install -y --no-install-recommends locales && \
    locale-gen en_US.UTF-8 && \
    echo "/usr/local/lib64" > /etc/ld.so.conf.d/local-lib64.conf && \
    ldconfig

# installing trac_ik with python bindings
RUN apt update && apt install -y -q --no-install-recommends \
    libboost-all-dev \
    libeigen3-dev \
    liborocos-kdl-dev \
    libnlopt-dev \
    libnlopt-cxx-dev

RUN pip3 install pytracik

# Install URDF/SRDF support for loading Fetch and other URDF-based robots
RUN apt-get update && apt-get install -y --no-install-recommends \
    liburdf-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libtinyxml2-dev \
    libconsole-bridge-dev

# Build srdfdom 0.4.2 from source (not in Ubuntu 22.04 repos)
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/ros-planning/srdfdom.git && \
    cd srdfdom && git checkout 0.4.2 && \
    printf 'cmake_minimum_required(VERSION 3.10)\nproject(srdfdom)\nfind_package(Boost REQUIRED)\nfind_package(console_bridge REQUIRED)\nfind_package(urdfdom_headers REQUIRED)\nfind_package(PkgConfig REQUIRED)\npkg_check_modules(TINYXML REQUIRED tinyxml)\nfind_path(URDF_INCLUDE_DIR urdf/model.h PATHS /usr/include /usr/local/include)\nfind_library(URDF_LIBRARY NAMES urdf)\ninclude_directories(include ${Boost_INCLUDE_DIRS} ${TINYXML_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS} ${URDF_INCLUDE_DIR})\nadd_definitions(-DlogError=CONSOLE_BRIDGE_logError -DlogWarn=CONSOLE_BRIDGE_logWarn -DlogInform=CONSOLE_BRIDGE_logInform -DlogDebug=CONSOLE_BRIDGE_logDebug)\nadd_library(srdfdom SHARED src/model.cpp)\ntarget_link_libraries(srdfdom ${TINYXML_LIBRARIES} ${console_bridge_LIBRARIES} ${URDF_LIBRARY} ${Boost_LIBRARIES})\ninstall(TARGETS srdfdom LIBRARY DESTINATION lib)\ninstall(DIRECTORY include/srdfdom DESTINATION include)\n' > CMakeLists.txt && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBoost_NO_BOOST_CMAKE=TRUE .. && \
    make -j $(nproc) && make install

# Build or_urdf OpenRAVE plugin (standalone, no ROS/catkin)
RUN mkdir -p ~/git; cd ~/git && \
    git clone https://github.com/AAIR-Lab/or_catkin.git && \
    cd or_catkin/or_urdf && \
    sed -i 's|#include <ros/package.h>|// removed: unused|' src/urdf_loader.cpp && \
    sed -i '19,27s/^/\/\//' src/urdf_loader.cpp && \
    sed -i '28s/^}/\/\/}/' src/urdf_loader.cpp && \
    sed -i '121,127s/^/\/\//' src/urdf_loader.cpp && \
    sed -i '128s/^}/\/\/}/' src/urdf_loader.cpp && \
    sed -i '252s/link_info->_t = /link_info->SetTransform(/' src/urdf_loader.cpp && \
    sed -i '253s/parent_joint->parent_to_joint_origin_transform) \* link_info->_t;/parent_joint->parent_to_joint_origin_transform) * link_info->GetTransform());/' src/urdf_loader.cpp && \
    sed -i '266s/geom_info->_t = /geom_info->SetTransform(/' src/urdf_loader.cpp && \
    sed -i '266s/);$/));/' src/urdf_loader.cpp && \
    sed -i '338s/geom_info->_t = /geom_info->SetTransform(/' src/urdf_loader.cpp && \
    sed -i '338s/);$/));/' src/urdf_loader.cpp && \
    sed -i '650,652c\            OpenRAVE::Transform st = sphere_info->GetTransform();\n            st.trans = collision_transform * OpenRAVE::Vector(\n                    sphere.center_x_, sphere.center_y_, sphere.center_z_);\n            sphere_info->SetTransform(st);' src/urdf_loader.cpp && \
    sed -i '777s/geom_info->_t = /geom_info->SetTransform(/' src/urdf_loader.cpp && \
    sed -i '777s/);$/));/' src/urdf_loader.cpp && \
    printf '#include <openrave/plugin.h>\n#include "urdf_loader.h"\nstruct URDFPlugin : public RavePlugin {\n    URDFPlugin() { _interfaces[OpenRAVE::PT_Module].push_back("urdf"); }\n    ~URDFPlugin() override {}\n    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override {\n        if (type == OpenRAVE::PT_Module && interfacename == "urdf") return OpenRAVE::InterfaceBasePtr(new or_urdf::URDFLoader(penv));\n        return OpenRAVE::InterfaceBasePtr();\n    }\n    const InterfaceMap& GetInterfaces() const override { return _interfaces; }\n    const std::string& GetPluginName() const override { return _pluginname; }\nprivate:\n    static const std::string _pluginname;\n    InterfaceMap _interfaces;\n};\nconst std::string URDFPlugin::_pluginname = "URDFPlugin";\nOPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() { return new URDFPlugin(); }\n' > src/or_urdf_plugin.cpp && \
    printf 'cmake_minimum_required(VERSION 3.10)\nproject(or_urdf)\nset(CMAKE_CXX_STANDARD 17)\nset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")\nfind_package(Boost REQUIRED COMPONENTS filesystem system)\nfind_package(PkgConfig REQUIRED)\npkg_check_modules(TINYXML REQUIRED tinyxml)\npkg_check_modules(TINYXML2 REQUIRED tinyxml2)\nfind_package(console_bridge REQUIRED)\nfind_path(OpenRAVE_INCLUDE_DIR openrave/openrave.h PATHS /usr/local/include/openrave-0.149)\nfind_library(OpenRAVE_LIBRARY NAMES openrave0.149 PATHS /usr/local/lib)\nfind_library(OpenRAVE_CORE_LIBRARY NAMES openrave0.149-core PATHS /usr/local/lib)\nfind_path(URDF_INCLUDE_DIR urdf/model.h PATHS /usr/include /usr/local/include)\nfind_library(URDF_LIBRARY NAMES urdf PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib)\nfind_path(SRDFDOM_INCLUDE_DIR srdfdom/model.h PATHS /usr/include /usr/local/include)\nfind_library(SRDFDOM_LIBRARY NAMES srdfdom PATHS /usr/local/lib /usr/lib/x86_64-linux-gnu)\nadd_definitions(-DlogError=CONSOLE_BRIDGE_logError -DlogWarn=CONSOLE_BRIDGE_logWarn -DlogInform=CONSOLE_BRIDGE_logInform -DlogDebug=CONSOLE_BRIDGE_logDebug -DOPENRAVE_DLL -DOPENRAVE_CORE_DLL)\ninclude_directories(src ${OpenRAVE_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${TINYXML_INCLUDE_DIRS} ${TINYXML2_INCLUDE_DIRS} ${URDF_INCLUDE_DIR} ${SRDFDOM_INCLUDE_DIR} ${console_bridge_INCLUDE_DIRS})\nadd_library(or_urdf SHARED src/urdf_loader.cpp src/catkin_finder.cpp src/or_urdf_plugin.cpp)\ntarget_link_libraries(or_urdf ${OpenRAVE_LIBRARY} ${OpenRAVE_CORE_LIBRARY} ${Boost_LIBRARIES} ${TINYXML_LIBRARIES} ${TINYXML2_LIBRARIES} ${URDF_LIBRARY} ${SRDFDOM_LIBRARY} ${console_bridge_LIBRARIES})\ninstall(TARGETS or_urdf LIBRARY DESTINATION lib/openrave0.149-plugins)\n' > CMakeLists.txt && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBoost_NO_BOOST_CMAKE=TRUE .. && \
    make -j $(nproc) && make install

# Install Python dependencies
WORKDIR /workspaces/
COPY requirements_py3.txt .
RUN pip3 install -r /workspaces/requirements_py3.txt
RUN rm -rf /workspaces/requirements_py3.txt

# Verify openravepy loads correctly
# X11, OpenGL, and Qt5 XCB runtime support for OpenRAVE viewer
RUN apt-get update && apt-get install -y \
    xauth xorg openbox htop openssh-client \
    libxcb-xinerama0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-render-util0 libxcb-xinput0 libxcb-xkb1 libxcb-cursor0 \
    libxkbcommon-x11-0 \
    libgl1-mesa-dri libgl1-mesa-glx libegl-mesa0 libgbm1 \
    dbus-x11 libdbus-1-3
# DBus machine-id needed by Qt5
RUN dbus-uuidgen > /etc/machine-id 2>/dev/null || true