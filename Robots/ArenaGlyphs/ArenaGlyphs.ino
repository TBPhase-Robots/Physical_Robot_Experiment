#include <M5Core2.h>

//  Stack screen width + height
#define WIDTH 320
#define HEIGHT 240

// 100: 25538311935
// 101: 24549434930
// 102: 27128378286
// 103: 27333326284
int number = 101;
uint64_t numbers[] = {25538311935, 24549434930, 27128378286, 27333326284};

//  Draws a thingy marker to the screen
void drawMarker(u_int64_t data, uint32_t background) {
  M5.lcd.clear();

  int size = HEIGHT / 10;
  int side_inset = (WIDTH - HEIGHT) / 2;

  M5.lcd.fillRect(0, 0, WIDTH, size, background);
  M5.lcd.fillRect(0, 0, size + side_inset, HEIGHT, background);

  M5.lcd.fillRect(0, HEIGHT - size, WIDTH, size, background);
  M5.lcd.fillRect(WIDTH - size - side_inset, 0, size + side_inset, HEIGHT, background);

  for (u_int64_t i = 0; i < 36; i++) {
    bool white = (data & ((u_int64_t)1 << i)) != 0;

    int x = side_inset + (i % 6 + 2) * size;
    int y = (i / 6 + 2) * size;
    if (white) {
      M5.lcd.fillRect(x, y, size, size, WHITE);
    }
  }
}

void setup() {
  //  Set up stack
  M5.begin();

  // //  Draw checkerboard marker to the screen
  drawMarker(numbers[number - 100], WHITE);
}

void loop() {
  M5.lcd.println(number);
  delay(10000000);
}
