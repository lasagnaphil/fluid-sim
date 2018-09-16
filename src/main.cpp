#include "App.h"

int main() {
    App app;
    app.init(Vector2i::create(800, 600));
    app.start();
    app.free();
    return 0;
}
