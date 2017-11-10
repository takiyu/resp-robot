#ifndef ROBOVIE_160624
#define ROBOVIE_160624

#include <boost/thread.hpp>
#include <map>
#include <string>
#include <vector>

#include "../../config.h"

// Robovie : Low serial layer class
class RobovieSerial {
public:
    RobovieSerial(bool degree_mode = true, bool safty_clamp = true,
                  bool debug_log = false);
    ~RobovieSerial() { close(); }

    void open();
    void close();

    void enableMotors();
    void disableMotors();

    void setMotor(int motor_idx, float value,
                  float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                  float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void setMotor(const std::string& label, float value,
                  float max_speed = ROBOVIE_MOTOR_DEFAULT_MAX_SPEED,
                  float accel = ROBOVIE_MOTOR_DEFAULT_ACCEL);
    void setMotorImmediately(int motor_idx, float value);
    void setMotorImmediately(const std::string& label, float value);

    float getMotorCurValue(int motor_idx);
    float getMotorCurValue(const std::string& label);
    float getMotorDstValue(int motor_idx);
    float getMotorDstValue(const std::string& label);

    void getMotorRange(int motor_idx, float& v_min, float& v_max);
    void getMotorRange(const std::string& label, float& v_min, float& v_max);
    void getMotorStep(int motor_idx, float& v_step);
    void getMotorStep(const std::string& label, float& v_step);

    bool isMotorMoving(int motor_idx) const;
    bool isMotorMoving(const std::string& label) const;
    bool isMotorMoving() const;

    void drawGlMotorUi();

protected:
    virtual void updateMotor();  // update method can be overridden

private:
    bool DEGREE_MODE, SAFTY_CLAMP, DEBUG_LOG;

    int fd;
    clock_t prev_clock;
    const double SERIAL_W_BLOCK_MS = 2.0;   // Update interval is controlled by
    const double SERIAL_R_BLOCK_MS = 23.0;  // sum of these parameters
    const static int BUFF_SIZE = 1024;
    char read_buffer[BUFF_SIZE];

    const int MEM_BYTE_OFFSET = 0x200800;
    // Memory container is int16_t
    const int MORTOR_MEM_IDX = 66;
    const int MAX_READ_MEM_SIZE = 8;

    // motor values are scaled (val * 256 = raw)
    const float MIN_DOWN_SPEED = 0.05f;
    std::vector<char> motor_moving;
    std::vector<float> motor_cur, motor_dst, motor_mid;
    std::vector<float> motor_speeds, motor_max_speeds, motor_accels;

    std::string motor_labels[ROBOVIE_N_MOTOR];

    int motor_ui_mode;

    void command(const std::string& line);
    void wcommand(int mem_idx, const std::vector<int16_t>& values);
    void rcommand(int mem_idx, int mem_size, std::vector<int16_t>& values);

    boost::thread* update_thread;
};
#endif
