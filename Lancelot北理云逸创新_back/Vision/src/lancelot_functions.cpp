#include "lancelot_functions.h"
#include <fstream>
#include <ctype.h>
#include <unistd.h>
#include <assert.h>

std::vector<std::string> state_str;
using namespace std;

void init()
{
    state_str.resize(40, "none");
    state_str[SG_LOW_CHECK] = "G_LC";
    state_str[SG_MID_CHECK] = "G_MC";
    state_str[SU_UP1] = "U_UP1";
    state_str[SU_HOLD] = "U_HOLD";
    state_str[SD_RETRY_UP] = "R_UP";
    state_str[SD_RETRY_UP_HOLD] = "R_HOLD";
    state_str[SD_CHECK_TARGET] = "CHK_TARGET";
    state_str[SD_FLY_TARGET] = "FLY_TARGET";
    state_str[TO_START] = "TO_START";

    state_str[SD_HOLD] = "D_HOLD";
    state_str[SD_MISS_SEARCH] = "D_MISS";
    state_str[SD_HOLD2] = "D_HOLD2";
    state_str[SD_HIGH_FAST_DOWN] = "D_FastD";
    state_str[SD_CIRCLE_SLOW_DOWN] = "D_CSD";
    state_str[SD_CIRCLE_HOLD] = "D_CHOLD";
    state_str[SD_CIRCLE_MID_DOWN] = "D_CMD";
    state_str[SD_CHECK_G] = "D_GC";
    state_str[SD_SHUT_DOWN] = "SHUT_DOWN";
    state_str[SD_SAFE] = "D_SAFE";
    state_str[HOLD_BACK] = "HOLD_BACK";
    state_str[BREAK] = "BREAK";
    state_str[SHUT] = "SHUT";
    state_str[GO_HOME] = "GO_HOME";
    state_str[SD_SCAN_9] = "SCAN_TARGETS";
    state_str[SD_SCAN_10] = "SCAN_TARGETS";
    state_str[SD_SCAN_11] = "SCAN_TARGETS";
    state_str[FLY_DIRECT] = "FLY_DIRECT";
}

std::string get_time()
{
    time_t time_ptr;
    time(&time_ptr);
    tm *ptm = gmtime(&time_ptr);
    char date[60] = {0};
    sprintf(date, "%d-%02d-%02d--%02d:%02d:%02d", (int)ptm->tm_year + 1900,
            (int)ptm->tm_mon + 1, (int)ptm->tm_mday, (int)ptm->tm_hour,
            (int)ptm->tm_min, (int)ptm->tm_sec);
    return std::string(date);
}

std::string expand_user(std::string path)
{
    if (not path.empty() and path[0] == '~')
    {
        assert(path.size() == 1 or path[1] == '/'); // or other error handling
        char const *home = getenv("HOME");
        if (home or ((home = getenv("USERPROFILE"))))
        {
            path.replace(0, 1, home);
        }
        else
        {
            char const *hdrive = getenv("HOMEDRIVE"),
                       *hpath = getenv("HOMEPATH");
            assert(hdrive); // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}