# AILab Responsive Robot Project #

When there is something you do not understand, I recommend you read the source code.

## Requirements ##
* premake5
* OpenCV3.1
* Boost
* PortAudio
* glfw3
* glew
* glm
* Torch7 (Fix paths for your environment) (optional)
* Gnuplot

## Testing Environment ##
(Please add your environment here.)
* Ubuntu 16.04
* Arch Linux (Please edit `./premake5.lua`)

## Build and Run ##
```bash
premake5 gmake
cd build
make  # or `make config=debug`
      # To compile with address sanitizer, build with `make config=release_asan
      #                                            or `make config=debug_asan`
      # To compile in parallel, use `-j` argument such as `-j4`.

sudo ./bin/release/main   # sudo is needed to open serial device.
```

## Configurations ##
Most of the important parameters are stored in `./src/config.cpp`.

### 3D Coordinates ###
All distance units are `cm`, and angle units are `deg`.
mR2 is the origin, and right hand system.
mR2's motor scale signs are also decided by right hand rotation.
They can be configured by `./src/config.cpp`, but the test is not enough yet.

```
         |                        (y)
         |                         +
  ---- [mR2] ----+ (x)             |
         |                       [Head]
         |                       [    ]
         +                       [Body]
        (z)                      [    ]
       [users]              ---- [    ] ----+ (z)
```

### WebCamera ###
```cpp
const int WEBCAMERA_ID = 0;                // Webcamera ID
const int WEBCAMERA_SIZE[2] = {640, 480};  // Webcamera captured image size
const int WEBCAMERA_FPS = 30;              // Webcamera FPS
const float WEBCAMERA_INTRINSIC_PARAMS[4] = {  // Intrinsic Parameters [fx, fy, cx, cy]
    7.0279330882735348e+02, 7.0279330882735348e+02, 3.1950000000000000e+02,
    2.3950000000000000e+02};
const cv::Point3f WEBCAMERA_POSITION(0.f, 54.f, -8.f);  // Webcamera absolute position
const cv::Point3f WEBCAMERA_ANGLES(5.f, -14.f, 0.f);    // Webcamera absolute angle
```

### Face Estimator ###
```cpp
const int N_FACE_MAX = 3;   // The number of faces to detect and track.
                            // It is recommended to set its value to the
                            // number of CPU core or less.
```

### Object Detector ###
Now, trained objects are red apple, green apple and hand.
To retrain object detectors, use training script `./scripts/train_object_detect_hog.sh` and `./scripts/train_object_classification_svm.sh.`

### Robovie Serial ###
`Robovie Serial` is low layer class to control robovie's motors.
```cpp
// == Raw value range of motors ==
// Motor values are clamped with the range for safety.
const float ROBOVIE_MOTOR_RANGES[ROBOVIE_N_MOTOR][2] = {
    // {raw_min, raw_max}
    {-87.f, 87.f},  // right arm 0
    {-87.f, 85.f},  // right arm 1
    ...

// == Scales and offsets ==
// The reference directions of the arms are [0, -1, 0], and others are [0, 0, 1]
const float ROBOVIE_MOTOR_NORM_PARAMS[ROBOVIE_N_MOTOR][2] = {
    // {scale, offset} :  deg_value * scale + offset = raw_value
    {-76.0f / 90.f, -38.f},   // right arm 0
    {-143.f / 90.f, -87.f},   // right arm 1
    ...
```
To decide the scales and offsets, `./bin/release/test_robovie_serial` may help you.

## TODO ##
- [x] Introduce common configuration file
- [ ] Introduce json configuration file
- [x] Fix position discrepancies
- [x] Add negative samples to object classification SVM
- [ ] Fix euler angles in `robovie_action.cpp`

## Notes ##
### WebCamera FPS ###
If `WEBCAMERA_FPS` is different from actual fps the capture will be slow,
because of grabbing thread interval.

### Dlib HOG Detector ###
To train object detector, execute the following command in build directory for
each object.
```bash
../scripts/train_object_detect_hog.sh <object_name>
../scripts/train_object_classification_svm.sh  # Edit class names
```
