#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

#include "imgui/imgui.h"

#include "robovie_serial.h"

namespace {

inline int OpenSerial(const char *dev_name) {
    int fd = open(dev_name, O_RDWR);
    if (fd < 0) {
        printf(" >> Failed to open\n");
    }
    return fd;
}

inline void CloseSerial(const int fd) { close(fd); }

inline void InitSerial(const int fd) {
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 100;
    const speed_t BAUD_RATE = B115200;
    cfsetispeed(&tio, BAUD_RATE);
    cfsetospeed(&tio, BAUD_RATE);
    tcsetattr(fd, TCSANOW, &tio);
}

inline int ReadSerial(const int fd, char *buff, const int buff_size) {
    int len = read(fd, buff, buff_size);
    if (len < 0) {
        printf(" >> Failed to read\n");
        return 0;
    }
    return len;
}

inline int WriteSerial(const int fd, const char *buff, const int buff_size) {
    int len = write(fd, buff, buff_size);
    if (len < 0) {
        std::cerr << "IO Error" << std::endl;
        return -1;
    }
    return len;
}

inline float Sign(float x) {
    if (x > 0.f) {
        return 1.f;
    } else if (x < 0.f) {
        return -1.f;
    } else {
        return 0.f;
    }
}

inline float UpdateToGoal(float cur, float goal, float step) {
    float dir = Sign(goal - cur);
    if (dir == 0.f) return cur;
    float next = cur + std::abs(step) * dir;
    if (0.f < dir && goal < next) return goal;
    if (dir < 0.f && next < goal) return goal;
    return next;
}

inline float ClampMotorRange(int motor_idx, float raw_value) {
    return std::min(std::max(raw_value, ROBOVIE_MOTOR_RANGES[motor_idx][0]),
                    ROBOVIE_MOTOR_RANGES[motor_idx][1]);
}

inline float ConvDeg2RawValue(int motor_idx, float deg_value) {
    return deg_value * ROBOVIE_MOTOR_NORM_PARAMS[motor_idx][0] +
           ROBOVIE_MOTOR_NORM_PARAMS[motor_idx][1];
}

inline float ConvRaw2DegValue(int motor_idx, float raw_value) {
    return (raw_value - ROBOVIE_MOTOR_NORM_PARAMS[motor_idx][1]) /
           ROBOVIE_MOTOR_NORM_PARAMS[motor_idx][0];
}

inline float ConvRaw2DegStep(int motor_idx, float raw_step) {
    return raw_step / std::abs(ROBOVIE_MOTOR_NORM_PARAMS[motor_idx][0]);
}

inline int GetMotorIdx(const std::string &label) {
    return ROBOVIE_MOTOR_IDXS.at(label);
}

void Split(const std::string &str, std::vector<std::string> &res, char sep) {
    res.clear();
    std::stringstream ss(str);
    std::string buf;
    while (std::getline(ss, buf, sep)) {
        if (!buf.empty()) {
            res.push_back(buf);
        }
    }
}
}

RobovieSerial::RobovieSerial(bool degree_mode, bool safty_clamp, bool debug_log)
    : update_thread(NULL),
      DEGREE_MODE(degree_mode),
      SAFTY_CLAMP(safty_clamp),
      DEBUG_LOG(debug_log) {
    // Set invalid file descriptor
    fd = -1;
    // Motor vectors
    motor_moving.resize(ROBOVIE_N_MOTOR, false);
    motor_cur.resize(ROBOVIE_N_MOTOR, 0.f);
    motor_dst.resize(ROBOVIE_N_MOTOR, 0.f);
    motor_mid.resize(ROBOVIE_N_MOTOR, 0.f);
    motor_speeds.resize(ROBOVIE_N_MOTOR, 0.f);
    motor_max_speeds.resize(ROBOVIE_N_MOTOR, 0.f);
    motor_accels.resize(ROBOVIE_N_MOTOR, 0.f);
    // Label
    std::map<std::string, int>::const_iterator itr = ROBOVIE_MOTOR_IDXS.begin();
    for (; itr != ROBOVIE_MOTOR_IDXS.end(); itr++) {
        motor_labels[itr->second] = itr->first;
    }
    // Read buffer
    for (int i = 0; i < BUFF_SIZE; i++) {
        read_buffer[i] = 0;
    }
    // UI
    motor_ui_mode = 0;
}

void RobovieSerial::open() {
    printf("* Open Robovie serial port (%s)\n", ROBOVIE_DEV_NAME.c_str());
    this->fd = OpenSerial(ROBOVIE_DEV_NAME.c_str());
    if (0 <= fd) {
        InitSerial(this->fd);
    }
    prev_clock = clock();  // set first clock

    // Set scale and offset
    this->command("w 200880 0f 00\r\n");
    this->command("w 200882 00 ff\r\n");

    // Read and set previous motor values
    printf("* Read and set previous Robovie motor values\n");
    for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
        std::vector<int16_t> values;
        rcommand(MORTOR_MEM_IDX + i, 1, values);
        if (values.size() != 0) {
            motor_cur[i] = values[0] / 255;
            motor_dst[i] = values[0] / 255;
        } else {
            motor_cur[i] = 0.f;
            motor_dst[i] = 0.f;
        }
    }

    // Enable servo motor
    this->enableMotors();

    // Start motor update thread
    update_thread = new boost::thread([&]() {
        printf("* Start a Robovie motor updating thread\n");
        while (true) {
            boost::this_thread::interruption_point();
            this->updateMotor();
        }
    });
}

void RobovieSerial::close() {
    if (update_thread != NULL) {
        printf("* Exit a Robovie motor updating thread\n");
        update_thread->interrupt();
        update_thread->join();
        delete update_thread;
        update_thread = NULL;
    }
    if (0 <= fd) {
        usleep(50 * 1000);  // for safety
        disableMotors();
        usleep(50 * 1000);  // for safety
        printf("* Close Robovie serial port\n");
        CloseSerial(fd);
        fd = -1;
    }
}

void RobovieSerial::enableMotors() {
    printf("* Enable Robovie motors\n");
    this->command("w 2009f6 01 00\r\n");
    this->command("w 2009f8 01 00\r\n");
}

void RobovieSerial::disableMotors() {
    printf("* Disable Robovie motors\n");
    this->command("w 2009f8 00 00\r\n");
    this->command("w 2009f6 00 00\r\n");
}

void RobovieSerial::setMotor(int motor_idx, float value, float max_speed,
                             float accel) {
    if (motor_idx < 0 || ROBOVIE_N_MOTOR <= motor_idx) {
        printf("* Invalid motor index (%d)\n", motor_idx);
        return;
    }
    if (DEGREE_MODE) {
        value = ConvDeg2RawValue(motor_idx, value);  // deg -> raw
    }
    float value_noclamp = value;
    if (SAFTY_CLAMP) {
        value = ClampMotorRange(motor_idx, value);  // raw clamp
    }
    float motor_diff = value - motor_cur[motor_idx];

    // Check the need to move
    if (((int16_t)motor_diff) == 0) return;

    // Correct direction
    float dir = Sign(motor_diff);
    max_speed *= dir * Sign(max_speed);
    accel *= dir * Sign(accel);

    // Set values
    motor_dst[motor_idx] = value;
    motor_mid[motor_idx] = (motor_cur[motor_idx] + value) * 0.5f;
    // motor_speeds[motor_idx] : no change
    motor_max_speeds[motor_idx] = max_speed;
    motor_accels[motor_idx] = accel;
    // Enable to move
    motor_moving[motor_idx] = true;
}

void RobovieSerial::setMotor(const std::string &label, float value,
                             float max_speed, float accel) {
    this->setMotor(GetMotorIdx(label), value, max_speed, accel);
}

void RobovieSerial::setMotorImmediately(int motor_idx, float value) {
    if (motor_idx < 0 || ROBOVIE_N_MOTOR <= motor_idx) {
        printf("* Invalid motor index (%d)\n", motor_idx);
        return;
    }
    if (DEGREE_MODE) {
        value = ConvDeg2RawValue(motor_idx, value);  // deg -> raw
    }
    float value_noclamp = value;
    if (SAFTY_CLAMP) {
        value = ClampMotorRange(motor_idx, value);  // raw clamp
    }
    // Set values
    motor_cur[motor_idx] = value;
    motor_dst[motor_idx] = value;
    // Set noclamp value
    motor_dst[motor_idx] = value_noclamp;
}

void RobovieSerial::setMotorImmediately(const std::string &label, float value) {
    this->setMotorImmediately(GetMotorIdx(label), value);
}

float RobovieSerial::getMotorCurValue(int motor_idx) {
    float v = motor_cur[motor_idx];
    if (DEGREE_MODE) {
        v = ConvRaw2DegValue(motor_idx, v);  // raw -> deg
    }
    return v;
}

float RobovieSerial::getMotorCurValue(const std::string &label) {
    return getMotorCurValue(GetMotorIdx(label));
}

float RobovieSerial::getMotorDstValue(int motor_idx) {
    float v = motor_dst[motor_idx];
    if (DEGREE_MODE) {
        v = ConvRaw2DegValue(motor_idx, v);  // raw -> deg
    }
    return v;
}

float RobovieSerial::getMotorDstValue(const std::string &label) {
    return getMotorDstValue(GetMotorIdx(label));
}

void RobovieSerial::getMotorRange(int motor_idx, float &v_min, float &v_max) {
    if (!SAFTY_CLAMP) {
        v_min = std::numeric_limits<float>::min();
        v_max = std::numeric_limits<float>::max();
    } else {
        v_min = ROBOVIE_MOTOR_RANGES[motor_idx][0];
        v_max = ROBOVIE_MOTOR_RANGES[motor_idx][1];
        if (DEGREE_MODE) {
            // raw -> deg
            v_min = ConvRaw2DegValue(motor_idx, v_min);
            v_max = ConvRaw2DegValue(motor_idx, v_max);
            // Fix min/max order
            if (v_max < v_min) {
                float tmp = v_min;
                v_min = v_max;
                v_max = tmp;
            }
        }
    }
}

void RobovieSerial::getMotorRange(const std::string &label, float &v_min,
                                  float &v_max) {
    return getMotorRange(GetMotorIdx(label), v_min, v_max);
}

void RobovieSerial::getMotorStep(int motor_idx, float &v_step) {
    if (!DEGREE_MODE) {
        v_step = 1.0;  // raw step
    } else {
        v_step = ConvRaw2DegStep(motor_idx, 1.0);
    }
}

void RobovieSerial::getMotorStep(const std::string &label, float &v_step) {
    return getMotorStep(GetMotorIdx(label), v_step);
}

bool RobovieSerial::isMotorMoving(int motor_idx) const {
    return motor_moving[motor_idx];
}

bool RobovieSerial::isMotorMoving(const std::string &label) const {
    return isMotorMoving(GetMotorIdx(label));
}

bool RobovieSerial::isMotorMoving() const {
    for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
        if (motor_moving[i]) return true;
    }
    return false;
}

void RobovieSerial::drawGlMotorUi() {
    // Common Window
    ImGui::Begin("Modules");
    // Header
    if (ImGui::CollapsingHeader("Robovie Serial",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Motor values");
        ImGui::SameLine();
        ImGui::RadioButton("Current", &this->motor_ui_mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Destination (Editable)", &this->motor_ui_mode, 1);
        ImGui::Columns(3);
        // Draw and update each motor
        for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
            // Motor Value
            float v;
            if (motor_ui_mode == 0) {
                v = this->getMotorCurValue(i);
            } else if (motor_ui_mode == 1) {
                v = this->getMotorDstValue(i);
            }
            // Draw slider
            if (motor_ui_mode == 0) {
                // Non-editable mode
                ImGui::DragFloat(motor_labels[i].c_str(), &v, 0);
            } else if (motor_ui_mode == 1) {
                // Editable mode
                float v_min, v_max, v_step;
                this->getMotorRange(i, v_min, v_max);
                this->getMotorStep(i, v_step);
                if (ImGui::DragFloat(motor_labels[i].c_str(), &v, v_step, v_min,
                                     v_max)) {
                    this->setMotor(i, v);
                }
            }
            // Draw horizontal line
            if (i == 11) {
                ImGui::Separator();
            }
            // Switch column
            if (i == 3 || i == 7 || i == 11 || i == 14) {
                ImGui::NextColumn();
            }
        }
    }
    ImGui::End();
}

void RobovieSerial::updateMotor() {
    // Update current motor status
    for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
        if (!motor_moving[i]) continue;
        float &cur = motor_cur[i];
        float &dst = motor_dst[i];
        float &mid = motor_mid[i];
        float &speed = motor_speeds[i];
        float &max_speed = motor_max_speeds[i];
        float &accel = motor_accels[i];
        if (DEBUG_LOG) {
            printf(
                "[motor %d] cur:%0.2f, mid:%0.2f, dst:%0.2f, speed:%0.2f,"
                " max_speed:%0.2f, accel:%f\n",
                i, cur, mid, dst, speed, max_speed, accel);
        }

        // Decide stage
        char stage;
        if (mid < dst) {
            if (cur < mid)
                stage = 0;  // speed up
            else if (cur < dst)
                stage = 1;  // speed down
            else
                stage = 2;  // finish
        } else {
            if (mid < cur)
                stage = 0;  // speed up
            else if (dst < cur)
                stage = 1;  // speed down
            else
                stage = 2;  // finish
        }

        // Update
        if (stage == 0) {
            // Speed up
            speed = UpdateToGoal(speed, max_speed, accel);
            cur = UpdateToGoal(cur, dst, speed);
        } else if (stage == 1) {
            // Correct acceleration (solve parabola)
            float corrected_accel = speed * speed / (2.f * (dst - cur));
            if (std::abs(accel) <= std::abs(corrected_accel)) {
                accel = (accel + corrected_accel) * 0.5f;
                // Speed down
                speed =
                    UpdateToGoal(speed, MIN_DOWN_SPEED * Sign(accel), accel);
            }
            cur = UpdateToGoal(cur, dst, speed);
        } else if (stage == 2) {
            // Finish
            motor_moving[i] = false;
            cur = dst;
            speed = 0.f;
            max_speed = 0.f;
            accel = 0.f;
        }
    }

    // Type convert: vector<float> -> vector<int16_t>
    std::vector<int16_t> values(ROBOVIE_N_MOTOR);
    const float int16_max = (float)std::numeric_limits<int16_t>::max();
    const float int16_min = (float)std::numeric_limits<int16_t>::min();
    for (int i = 0; i < ROBOVIE_N_MOTOR; i++) {
        float value = motor_cur[i] * 256.f;  // scaling
        values[i] = (int16_t)std::min(std::max(value, int16_min), int16_max);
    }
    // Send
    this->wcommand(MORTOR_MEM_IDX, values);
}

void RobovieSerial::command(const std::string &line) {
    // Blocking
    double elapse_ms = (double)(clock() - prev_clock) / CLOCKS_PER_SEC * 1000.0;
    double sleep_ms = SERIAL_W_BLOCK_MS - elapse_ms;
    if (sleep_ms > 0) {
        usleep(sleep_ms * 1000.0);
    }
    if (0 <= fd) {
        // Write
        WriteSerial(this->fd, line.c_str(), line.length());
        // Wait for returning
        usleep(SERIAL_R_BLOCK_MS * 1000.0);
        // Read
        ReadSerial(this->fd, read_buffer, BUFF_SIZE);
    } else {
        usleep(SERIAL_R_BLOCK_MS * 1000.0);
    }
    // Set current clock
    prev_clock = clock();
}

void RobovieSerial::wcommand(int mem_idx, const std::vector<int16_t> &values) {
    // Command string (the container is 2 byte (int16_t))
    int mem_byte_number = MEM_BYTE_OFFSET + mem_idx * 2;
    std::stringstream com_ss;
    com_ss << "w " << std::hex << mem_byte_number;
    // hex array
    for (int i = 0; i < values.size(); i++) {
        // Convert value to hex string
        std::stringstream hex_ss;
        hex_ss << std::setfill('0') << std::setw(4) << std::hex << values[i];
        std::string hex_str = hex_ss.str();
        com_ss << " " << hex_str[2] << hex_str[3];
        com_ss << " " << hex_str[0] << hex_str[1];
    }
    // tail
    com_ss << "\r\n";

    // Send
    this->command(com_ss.str());
}

void RobovieSerial::rcommand(int mem_idx, int mem_size,
                             std::vector<int16_t> &values) {
    values.clear();
    if (MAX_READ_MEM_SIZE < mem_size) {
        printf("* Failed to read Robovie memory (size is too large: %d < %d\n",
               MAX_READ_MEM_SIZE, mem_size);
        return;
    }

    // Command string (the container is 2 byte (int16_t))
    int mem_byte_number = MEM_BYTE_OFFSET + mem_idx * 2;
    std::stringstream com_ss;
    com_ss << "r " << std::hex << mem_byte_number << " ";
    com_ss << std::setfill('0') << std::setw(2) << std::hex << (mem_size * 2);
    com_ss << "\r\n";
    // Send
    this->command(com_ss.str());

    if (0 <= fd) {
        // Returned string
        std::string ret_str(read_buffer);

        // Remove header
        std::stringstream header_ss;
        header_ss << "#" << std::hex << mem_byte_number;
        int header_idx = ret_str.find(header_ss.str());
        if (header_idx == -1) {
            printf("* Failed to read Robovie memory (no header)\n");
            return;
        }
        ret_str = ret_str.substr(header_idx + header_ss.str().size());

        // Check data size (`ret_str` may be " 23 01" + ... + " \r\n")
        if (ret_str.size() < mem_size * 6) {
            printf("* Failed to read Robovie memory (invalid returning)\n");
            return;
        }

        // Parse
        values.resize(mem_size);
        for (int i = 0; i < mem_size; i++) {
            std::stringstream ss;
            ss << ret_str[6 * i + 4] << ret_str[6 * i + 5];
            ss << ret_str[6 * i + 1] << ret_str[6 * i + 2];
            uint16_t v;
            ss >> std::hex >> v;
            // Cast for negative value
            values[i] = reinterpret_cast<int16_t &>(v);
        }
    }
}
