#include "kitemath.h"

// acos function continuously extended beyond -1 and 1.
float safe_acos(float number_more_or_less_between_one_and_minus_one) {
    return (fabs(number_more_or_less_between_one_and_minus_one) < 1) ? acos(
            number_more_or_less_between_one_and_minus_one) : 0;
    return 0;
}

int smallpow(int x, int p) {
    int ret = 1;
    for (int i = 0; i < p; i++) {
        ret *= x;
    }
    return ret;
}
