from conans import ConanFile, CMake, tools
import os

__version__ = "0.0.0"

class SpectacularAIConan(ConanFile):
    name = "SpectacularAI"
    version = __version__
    license = "Tevel"
    author = "Gal gal@tevel-tech.com"
    description = "SpectacularAI"
    topics = ("HAL", "proto", "", "SpectacularAI")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = ("shared=False", "libusb:shared=True", \
                       "utilities:use_ros=False", "cmake:with_openssl=False" ,\
                       "opencv:contrib=True", \
                       "opencv:with_ffmpeg=True", \
                       "opencv:with_gtk=True", \
                       "gtk:version=2"
                       )
    generators = "cmake"
    exports_sources = "*"
    requires =  "utilities/3.3.12", \
                "bzip2/1.0.8", \
                "sharedMemory/1.7.2", \
                "cameraParameters/0.1.7", \
                "systemConfig/4.0.5", \
                "jsoncpp/1.8.4@tevel/stable", \
                "eigen/3.3.9", \
                "TevelRosMessages/0.3.9", \
                "pkgconf/1.7.3", \
                "opencv/3.4.12@tevel/ffmpeg", \
                "glib/2.68.0", \
                "gtest/1.8.1", \
                "openssl/1.1.1m", \
                "notifications/1.0.5", \
                "TevelStatusCodes/0.0.29", \
                "controlTowerClient/1.0.0-dev-3", \
                "UvcDataInterface/1.0.0", \
                "utilities/3.3.12", \
                "bzip2/1.0.8", \
                "sharedMemory/1.7.2", \
                "cameraParameters/0.1.7", \
                "systemConfig/4.0.5", \
                "jsoncpp/1.8.4@tevel/stable", \
                "eigen/3.3.9", \
                "TevelRosMessages/0.3.9", \
                "pkgconf/1.7.3", \
                "opencv/3.4.12@tevel/ffmpeg", \
                "glib/2.68.0", \
                "gtest/1.8.1", \
                "openssl/1.1.1m", \
                "notifications/1.0.5", \
                "TevelStatusCodes/0.0.29", \
                "controlTowerClient/1.0.0-dev-3", \
                "zlib/1.2.11", \
                "boost/1.65.1@conan/stable"
    build_requires = "cmake/3.23.1"

    def build(self):
        cmake = CMake(self)
        if self.settings.arch == 'armv8':
            cmake.definitions["ARM_ARCH"] = True
        cmake.configure(source_folder=".")
        with open("./include/version.h", "w") as version_file:
            version_file.write("#define VERSION \"" + __version__ + "\"")
        cmake.build()

    def package(self):
        self.copy("*.h*", dst="include", keep_path=False)
        self.copy("*.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.so", dst="lib", keep_path=False)
        self.copy("*.dylib", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)
        self.copy("bin/*", dst="bin", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["frontCameraDriver"]

