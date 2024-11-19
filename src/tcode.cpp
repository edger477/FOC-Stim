#include "tcode.h"

size_t find_delimitter(const char *buffer, size_t start, size_t end)
{
    const char delimitters[] = ",; \r\n";
    for (size_t i = start; i < end; i++)
    {
        for (size_t j = 0; j < sizeof(delimitters); j++)
        {
            if (buffer[i] == delimitters[j])
            {
                return i;
            }
        }
    }
    return -1;
}

size_t count_digits(const char *buffer, size_t start, size_t end)
{
    for (size_t i = start; i < end; i++)
    {
        if (buffer[i] < '0' || buffer[i] > '9')
        {
            return i - start;
        }
    }
    return end - start;
}

int parse_tcode(const char *buffer, size_t len, TCodeCommand *out)
{
    memset(out, 0, sizeof(TCodeCommand));

    int index = 0;
    while (index + 3 <= len)
    {
        char ch = buffer[index];
        // first character of the command must be A-Z
        if (ch > 'Z' || ch < 'A')
        {
            index += 1;
            continue;
        }
        out->id.type = ch;

        // read channel num, 1 digit
        char channel_ch = buffer[index + 1];
        if (channel_ch < '0' || channel_ch > '9')
        {
            index += 1;
            continue;
        }
        out->id.num = channel_ch - '0';

        // read value, n digits
        int digits = count_digits(buffer, index + 2, len);
        if (digits == 0)
        {
            index += 2;
            continue;
        }
        int value = 0;
        int ret = sscanf(buffer + index + 2, "%d", &value);
        out->value = value / powf(10, digits);
        index += 2 + digits;

        // read I?
        int interval = 0;
        if (index + 2 <= len)
        {
            char nextchar = buffer[index];
            if (nextchar == 'I' || nextchar == 'i')
            {
                int digits = count_digits(buffer, index + 1, len);
                if (digits == 0)
                {
                    // malformed packet?
                    index += 1;
                }
                else
                {
                    int ret = sscanf(buffer + index + 1, "%d", &interval);
                    out->interval = interval;
                    index += 1 + digits;
                }
            }
        }

        return index;
    }
    return -1;
}

void print_tcode_command(TCodeCommand cmd)
{
    // if (cmd.interval) {
    // Serial.printf("%c%d:%f:%d", cmd.cmd, cmd.channel, (double)cmd.value, cmd.interval);
    // } else {
    // Serial.printf("%c%d:%f", cmd.cmd, cmd.channel, (double)cmd.value);
    // }
}

bool TCode::update_from_serial()
{
    bool dirty = false;
    while (Serial.available())
    {
        char c = Serial.read();
        if (c != -1)
        {
            serial_buffer[buffer_index++] = c;
            // start parsing on newline
            if (c == '\r' || c == '\n' || buffer_index >= 255)
            {
                dirty |= parse_data(serial_buffer, buffer_index);
                memset(serial_buffer, 0, sizeof(serial_buffer) / sizeof(serial_buffer[0]));
                buffer_index = 0;
            }
        }
    }
    return dirty;
}

bool TCode::parse_data(const char *buffer, size_t len)
{
    TCodeCommand cmd;
    int ret = 0;
    int index = 0;
    bool dirty = false;
    while (1)
    {
        ret = parse_tcode(buffer + index, len - index, &cmd);
        if (ret == -1)
        {
            return dirty;
        }

        dirty |= send_command(cmd);
        index += ret;
    }
}

bool TCode::send_command(TCodeCommand cmd)
{
    bool dirty = false;

    unsigned long interval_us;
    if (cmd.interval == 0)
    {
        interval_us = 30000;
    }
    else
    {
        interval_us = (unsigned long)cmd.interval * 1000;
    }

    unsigned i = 0;
    for (i = 0; i < axes_num; i++)
    {
        if (axes[i].id == cmd.id)
        {
            dirty |= axes[i].value1 != cmd.value;
            axes[i].set(micros(), cmd.value, interval_us);
            break;
        }
    }

    // if (dirty) {
    //     Serial.print("parsed command: ");
    //     Serial.print(cmd.id.as_string());
    //     Serial.print(":");
    //     Serial.print(cmd.value);
    //     if (cmd.interval) {
    //         Serial.print("~");
    //         Serial.print(cmd.interval);
    //     }
    //     Serial.println();
    // }
    return dirty;
}
