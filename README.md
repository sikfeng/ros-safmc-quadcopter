# ROS SAFMC Quadcopter

# Dependencies

Need `ros-noetic`

As of August 20 2021, when installing `mavros` using `yay -S ros-noetic-mavros`, I get the following error

```
/usr/include/log4cxx/boost-std-configuration.h: At global scope:
/usr/include/log4cxx/boost-std-configuration.h:10:18: error: ‘shared_mutex’ in namespace ‘std’ does not name a type
   10 |     typedef std::shared_mutex shared_mutex;
      |                  ^~~~~~~~~~~~
/usr/include/log4cxx/boost-std-configuration.h:10:13: note: ‘std::shared_mutex’ is only available from C++17 onwards
   10 |     typedef std::shared_mutex shared_mutex;
      |             ^~~
```

To fix this add `-DCMAKE_CXX_STANDARD=17 \` to line 71 of `PKGBUILD` and build manually.

# Compiling

Run `source /opt/ros/noetic/setup.zsh` before starting


