#include "App.h"

#ifdef __MINGW32__
#undef main
#endif

int main(int argc, char* argv[]) {
    App app;
    app.init(vec2i {1600, 900});
    app.start();
    app.free();
    return 0;
}
