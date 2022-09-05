#include <M5Core2.h>

//  Stack screen width + height
#define WIDTH 320
#define HEIGHT 240
//   uint64_t         uint32_t
// 100: 25538311935, 4x4: 31342
// 101: 24549434930,  ||: 46314
// 102: 27128378286,  ||: 60751
// 103: 27333326284,  ||: 64743
int number = 103;
uint32_t numbers[] = {31342, 46314, 60751, 64743};

//  Draws a thingy marker to the screen
void drawMarkerSmall(u_int32_t data, uint32_t background) {
  M5.lcd.clear();

  int size = HEIGHT / 8; //used to be 10
  int side_inset = (WIDTH - HEIGHT) / 2;

  M5.lcd.fillRect(0, 0, WIDTH, size, background);
  M5.lcd.fillRect(0, 0, size + side_inset, HEIGHT, background);

  M5.lcd.fillRect(0, HEIGHT - size, WIDTH, size, background);
  M5.lcd.fillRect(WIDTH - size - side_inset, 0, size + side_inset, HEIGHT, background);

  for (u_int64_t i = 0; i < 16; i++) {
    bool white = (data & ((u_int32_t)1 << i)) != 0;

    int x = side_inset + (i % 4 + 2) * size;
    int y = (i / 4 + 2) * size;
    if (white) {
      M5.lcd.fillRect(x, y, size, size, WHITE);
    }
  }

//  for (u_int64_t i = 0; i < 36; i++) {
//    bool white = (data & ((u_int64_t)1 << i)) != 0;
//
//    int x = side_inset + (i % 6 + 2) * size;
//    int y = (i / 6 + 2) * size;
//    if (white) {
//      M5.lcd.fillRect(x, y, size, size, WHITE);
//    }
//  }
}

//  Draws a large thingy marker to the screen
void drawMarker(u_int32_t data, uint32_t background) {

  M5.lcd.clear();

  int size = HEIGHT / 7;
  int side_inset = (WIDTH - HEIGHT) / 2;

  // Draw borders in the background colour
  M5.lcd.fillRect(0, 0, WIDTH, size/2, background);
  M5.lcd.fillRect(0, 0, side_inset+(size/2), HEIGHT, background);
  M5.lcd.fillRect(0, HEIGHT - (size/2), WIDTH, size/2, background);
  M5.lcd.fillRect(WIDTH - side_inset - (size/2), 0, side_inset+(size/2), HEIGHT, background);

  // Read the binary encoding of the marker and draw it to the screen
    for (u_int32_t i = 0; i < 16; i++) {
    bool white = (data & ((u_int32_t)1 << i)) != 0;

    int x = side_inset + (i % 4 + 1.5) * size;
    int y = (i / 4 + 1.5) * size;
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
