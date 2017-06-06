//
// Created by han on 2016/5/13.
//

#ifndef AROBOT_TYPEDEF_H
#define AROBOT_TYPEDEF_H

#include <cstdint>

namespace arobot{
    namespace core {

        union byte_2 {
            uint8_t byte[2];
            uint16_t udata;
            int16_t data;
        };

        union byte_4 {
            uint8_t byte[4];
            uint32_t udata;
            int32_t data;
            float fdata;
        };

        union byte_8 {
            uint8_t byte[8];
            uint64_t udata;
            int64_t data;
            double fdata;
        };

    }
}

#endif //AROBOT_TYPEDEF_H
