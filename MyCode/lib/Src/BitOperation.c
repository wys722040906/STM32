#include "BitOperation.h"

int count_ones(unsigned char byte) {
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            count++;
        }
    }
    return count;
}