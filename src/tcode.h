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

class TCode
{
public:
    TCode(TCodeAxis *axes, unsigned axes_num)
        : axes(axes), axes_num(axes_num)
    {
    }

    bool update_from_serial();

    bool parse_data(const char *buffer, size_t len);

    bool send_command(TCodeCommand cmd);

    TCodeAxis *axes;
    unsigned axes_num;

    char serial_buffer[256];
    int buffer_index = 0;
};

#endif // FOCSTIM_TCODE_H