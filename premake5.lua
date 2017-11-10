-- premake5.lua
--
-- *** Please fix this script for your environment. ***
--
exec_sources = {
  "./src/main.cpp",
  "./src/tests/test_camera_face.cpp",
  "./src/tests/test_camera_object.cpp",
  "./src/tests/test_camera_pose.cpp",
  "./src/tests/test_camera_scene.cpp",
  "./src/tests/test_enum_pa_devices.cpp",
  "./src/tests/test_models_sensors.cpp",
  "./src/tests/test_models_projector.cpp",
  "./src/tests/test_plotter.cpp",
  "./src/tests/test_robovie_action.cpp",
  "./src/tests/test_robovie_serial.cpp",
  "./src/tests/test_shapes_viewer.cpp",
  "./src/tests/test_vad.cpp",
  "./src/tests/test_voice_echo.cpp",
  "./src/tests/test_voice_recognition.cpp",
  "./src/tests/test_voice_synthesis.cpp",
  "./src/tools/tool_camera_capture.cpp",
  "./src/tools/tool_optimize_weights.cpp",
  "./src/tools/tool_train_hog.cpp",
  "./src/tools/tool_train_svm.cpp",
}

-- Torch 7 option
newoption {
  trigger     = "with-torch",
  description = "Flag for building without Torch 7"
}
if _OPTIONS["with-torch"] then
  print('>> With Torch')
  WITH_TORCH = true
else
  print('>> No Torch')
  defines { "NO_TORCH" }
  WITH_TORCH = false
end

workspace "RespRobotWorkspace"
  language "C++"
  configurations { "release", "debug", "release_asan" }
  basedir "build"
  flags { "C++11" }

--   toolset "clang"

  includedirs { "/usr/local/include" }
  libdirs { "/usr/local/lib" }
  libdirs { "./build/bin/release" }

  -- Common includes and links
  includedirs {  -- set also `.syntastic_cpp_config` for vim
    "/usr/include/boost",
    "./3rdParty",
    "./3rdParty/dlib",
    "./3rdParty/OpenFace/lib/FaceAnalyser/include",
    "./3rdParty/OpenFace/lib/LandmarkDetector/include",
    "./3rdParty/webrtc/common_audio/vad/include",
    "./3rdParty/julius/libjulius/include",
    "./3rdParty/julius/libsent/include",
    "./3rdParty/hts_engine/include",
    "./3rdParty/open_jtalk",
    "./3rdParty/imgui",
    "./3rdParty/pose-hg",
    "./3rdParty/tiny_obj_loader",
    "./3rdParty/gnuplot-iostream",
    "./3rdParty/picojson",
  }
  links { "boost_system", "boost_filesystem", "boost_thread",
          "boost_iostreams" }
  links { "opencv_core", "opencv_highgui", "opencv_objdetect",
          "opencv_videoio", "opencv_imgproc", "opencv_calib3d",
          "opencv_imgcodecs", }  -- cv 3.1
  links { "GLEW", "glfw", "GLU", "GL" }
  links { "portaudio", "pthread", "sndfile", "jpeg", "tbb", "png" }
  links { "X11", "Xrandr", "Xi", "Xxf86vm", "Xcursor", "Xinerama" }

  defines { "DLIB_JPEG_SUPPORT", "DLIB_PNG_SUPPORT" } -- dlib common
  defines { "DLIB_HAVE_SSE2", "DLIB_HAVE_SSE3", "DLIB_HAVE_SSE41" } -- SSE4
  vectorextensions "SSE4.1"

  -- Debug/Release Configuration
  filter "configurations:debug"
    defines { "DEBUG", "_DEBUG" }
    flags { "Symbols" }
  filter "configurations:release or release_asan"
    defines { "NDEBUG" }
    optimize "Full"
    -- with address sanitizer
    filter "configurations:release_asan"
      linkoptions { "-fsanitize=address" }
      buildoptions { "-fsanitize=address" }

-- [3rd/Library] dlib
project "dlib"
  kind "SharedLib"
  files { "./3rdParty/dlib/dlib/all/source.cpp" }

-- [3rd/Library] OpenFace
project "openface"
  kind "SharedLib"
  files { "./3rdParty/OpenFace/lib/**.cpp" }

-- [3rd/Library] WebRTC VAD
project "webrtc_vad"
  kind "SharedLib"
  files { "./3rdParty/webrtc/**.c" }
  defines { "WEBRTC_POSIX" }

-- [3rd/Library] Julius
project "julius"
  kind "SharedLib"
  if os.execute("cd ./3rdParty/julius/libjulius && sh configure") ~= 0 or
     os.execute("cd ./3rdParty/julius/libsent && " ..
                "sh configure --with-mictype=portaudio") ~= 0 then
    return
  end
  files { "./3rdParty/julius/libjulius/**.c" }
  files { "./3rdParty/julius/libsent/**.c" }
  removefiles { "./3rdParty/julius/libsent/src/adin/**" }
  files { "./3rdParty/julius/libsent/src/adin/adin_file.c" }
  files { "./3rdParty/julius/libsent/src/adin/adin_portaudio.c" }
  files { "./3rdParty/julius/libsent/src/adin/adin_sndfile.c" }
  files { "./3rdParty/julius/libsent/src/adin/adin_tcpip.c" }
  files { "./3rdParty/julius/libsent/src/adin/ds48to16.c" }
  files { "./3rdParty/julius/libsent/src/adin/zmean.c" }
  files { "./3rdParty/julius/libsent/src/adin/zc-e.c" }

-- [3rd/Library] OpenJTalk
project "open_jtalk"
  kind "SharedLib"
  defines {
    "HAVE_SYS_TYPES_H", "HAVE_SYS_STAT_H", "HAVE_FCNTL_H", "HAVE_STRING_H", 
    "HAVE_SYS_MMAN_H", "HAVE_UNISTD_H", "HAVE_STDINT_H", "HAVE_DIRENT_H",
    "CHARSET_UTF_8",
--     "DIC_VERSION=\"102\"", 'PACKAGE=\\\"open_jtalk\\\"', "VERSION=\\\"1.09\\\"",  -- archlinux
    "DIC_VERSION=102", 'PACKAGE=\"open_jtalk\"', "VERSION=\"1.09\"",  -- ubuntu
  }
  buildoptions { "-fpermissive" }
  includedirs { "./3rdParty/open_jtalk/mecab/src", "./3rdParty/open_jtalk/*" }
  files { "./3rdParty/open_jtalk/**.c", "./3rdParty/open_jtalk/**.cpp" }
  removefiles { "./3rdParty/open_jtalk/mecab/src/mecab-dict-index.cpp" }
  files { "./3rdParty/hts_engine/lib/**.c" }  -- HTS Engine

-- [3rd/Library] ImGui
project "imgui"
  kind "SharedLib"
  files { "./3rdParty/imgui/**.cpp" }

-- [3rd/Tool] Dlib imglab
project "tool_imglab"
  kind "ConsoleApp"
  files { "./3rdParty/dlib_imglab/src/*.cpp" }
  links { "dlib", "jpeg" }

-- [3rd/Library] pose-hg
if WITH_TORCH then -- Torch 7
project "pose_hg"
  kind "SharedLib"
  files { "./3rdParty/pose-hg/pose-hg.cpp" }
  -- Torch 7
  includedirs { "/home/takiyu/Projects/torch/install/include" }  -- TODO Configurable Torch path
  libdirs { "/home/takiyu/Projects/torch/install/lib" }
  links { "TH", "luaT", "luajit" }
end

-- [Library] My (TODO rename)
project "my"
  kind "SharedLib"
  files { "./src/**.cpp", }
  removefiles { exec_sources }
  links { "dlib", "openface", "webrtc_vad", "julius", "open_jtalk", "imgui" }

-- Executable projects
for i, path in ipairs(exec_sources) do
  name = string.match(path, ".*/(.-)%.%w-$")  -- "dir/name.ext"
  if not name then
    name = string.match(path, "(.-)%.%w-$")  -- "name.ext"
  end

  project( name )
    kind "ConsoleApp"
    links { "dlib", "openface", "webrtc_vad", "julius", "open_jtalk", "imgui",
            "my" }
    buildoptions { "-fopenmp" }
    files { path }
    if WITH_TORCH then -- Torch 7
      links { "pose_hg" }
    end
end
