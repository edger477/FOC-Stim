#ifndef FOCSTIM_TCODE_H
#define FOCSTIM_TCODE_H

#include <Arduino.h>

struct TCodeAxisIdentifier
{
    TCodeAxisIdentifier() {}
    TCodeAxisIdentifier(char type, uint8_t num)
    {
        this->type = type;
        this->num = num;
    }
    TCodeAxisIdentifier(const char id[2])
    {
        this->type = id[0];
        this->num = id[1] - '0';
    }

    bool operator==(TCodeAxisIdentifier &other)
    {
        return type == other.type && num == other.num;
    };

    String as_string()
    {
        char c_str[3] = {
            type,
            (char)(num + '0'),
            '\0',
        };
        return String(c_str);
    }

    char type; // 'L' or 'V'...
    char num;  // 0-9, stored as an int.
};

struct TCodeCommand
{
    TCodeAxisIdentifier id;
    float value;
    int interval; // in milis
};

struct TCodeAxis
{
    TCodeAxis(TCodeAxisIdentifier id, float init, float min_value, float max_value)
        : id(id), T0(0), T1(0), value0(0), value1((init - min_value) / (max_value - min_value)), min_value(min_value), max_value(max_value)

    {
    }

    TCodeAxisIdentifier id;
    unsigned long T0;
    unsigned long T1;
    float value0;
    float value1;
    float min_value;
    float max_value;

    float get_raw(unsigned long ts)
    {
        float p;
        if (T1 == T0)
        {
            p = 1;
        }
        else
        {
            p = float(ts - T0) / float(T1 - T0);
            p = max(min(p, 1.f), 0.f);
        }
        return value0 + p * (value1 - value0);
    }

    float get_remap(unsigned long ts)
    {
        return min_value + get_raw(ts) * (max_value - min_value);
    }

    float get_speed(unsigned long ts)
    {
        if (ts < T0)
        {
            return 0;
        }
        else if (ts > T1)
        {
            return 0;
        }
        return (value1 - value0) / (T1 - T0) * 1000000;
    }

    void set(unsigned long ts, float value, unsigned long interval_micros)
    {
        value0 = this->get_raw(0);
        value1 = value;
        T0 = ts;
        T1 = ts + interval_micros;
    }
};

struct TCodeDeviceCommand {
    TCodeDeviceCommand(const char* command_string, std::function<void()> fn)
        : command_string(command_string)
        , fn(fn)
    {}

    const char* command_string;
    std::function<void()> fn;
};

class TCode
{
public:
    TCode(TCodeAxis *axes, unsigned axes_num,
            TCodeDeviceCommand* device_commands, unsigned device_commands_num)
        : axes(axes), axes_num(axes_num)
        , device_commands(device_commands), device_commands_num(device_commands_num)
    {
    }

    bool update_from_serial();

    bool parse_single_line(const char *buffer, size_t len);

    bool update_axis(TCodeCommand cmd);
    bool execute_device_command(TCodeCommand cmd);

    // Look in the datastream for the first TCode command
    // if tcode command was found, return the number of bytes parsed.
    // else, return -1.
    int try_parse_tcode_command(const char *buffer, size_t len, TCodeCommand *out);


    TCodeAxis *axes;
    unsigned axes_num;
    TCodeDeviceCommand *device_commands;
    unsigned device_commands_num;

    char serial_buffer[256];
    int buffer_index = 0;
};

#endif // FOCSTIM_TCODE_H