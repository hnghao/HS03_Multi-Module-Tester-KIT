#ifndef DISPLAY_H
#define DISPLAY_H

// Số item mỗi trang trên menu chính
const uint8_t MAIN_ITEMS_PER_PAGE = 3;

// In 1 dòng, đảm bảo:
// 1) Xoá sạch 20 ký tự trên dòng
// 2) Chỉ in tối đa 20 ký tự từ text
void lcdPrintLine(uint8_t row, const char *text) {
  // B1: xoá dòng (20 space)
  lcd.setCursor(0, row);
  lcd.print("                    "); // 20 spaces

  // B2: in nội dung mới, cắt tối đa 20 ký tự
  if (text && text[0] != '\0') {
    char buf[21];
    strncpy(buf, text, 20);
    buf[20] = '\0';

    lcd.setCursor(0, row);
    lcd.print(buf);
  }
}

// --------------------
// Header (dòng 0)
// --------------------
// Main menu:  "MENU a/b  xxxs"
//   - a: số thứ tự chức năng hiện tại (1..MAIN_MENU_COUNT)
//   - b: tổng số chức năng trong chương trình (MAIN_MENU_COUNT)
// I2C submenu: "I2C  a/b  xxxs"
// Mode khác:  "<headerLabel>   xxxs"
void updateHeaderRow() {
  // Nếu header bị tắt (ví dụ khi vào PS2) thì không vẽ gì lên dòng 0
  if (!headerEnabled) {
    return;
  }
  char line[21];

  if (appState == STATE_MENU) {
    if (currentLevel == LEVEL_MAIN) {
      // Vị trí hiện tại trong toàn bộ danh sách chức năng
      int currentPos = currentMainIndex + 1;   // 1..MAIN_MENU_COUNT
      int totalFunc  = MAIN_MENU_COUNT;        // b

      // Ví dụ: "MENU 5/15 120s"
      snprintf(line, sizeof(line), "MENU %d/%d  %3ds",
               currentPos, totalFunc, countdownRemaining);
      lcdPrintLine(0, line);

    } else if (currentLevel == LEVEL_I2C_SUB) {
      // Submenu I2C: dùng số thứ tự / tổng số mục trong I2C
      int currentPos = currentI2CIndex + 1;    // 1..I2C_MENU_COUNT
      int totalFunc  = I2C_MENU_COUNT;

      // Ví dụ: "I2C  2/3  118s"
      snprintf(line, sizeof(line), "I2C  %d/%d  %3ds",
               currentPos, totalFunc, countdownRemaining);
      lcdPrintLine(0, line);

    } else {
      // Phòng hờ (ít khi vào)
      snprintf(line, sizeof(line), "MENU ?/?  %3ds", countdownRemaining);
      lcdPrintLine(0, line);
    }
  } else {
    // Các mode khác: giữ kiểu header cũ
    // "<headerLabel>   118s"
    snprintf(line, sizeof(line), "%-12s%4ds", headerLabel, countdownRemaining);
    lcdPrintLine(0, line);
  }
}

// --------------------
// MAIN MENU (LEVEL_MAIN)
// --------------------
// Dòng 0: header (MENU a/b + countdown)
// Dòng 1–3: item trong TRANG, có con trỏ '>'
void printMainMenuItem() {
  updateHeaderRow();  // vẽ dòng 0

  int pageIndex   = currentMainIndex / MAIN_ITEMS_PER_PAGE;   // 0-based
  int startIndex  = pageIndex * MAIN_ITEMS_PER_PAGE;
  int endIndex    = startIndex + MAIN_ITEMS_PER_PAGE;
  if (endIndex > MAIN_MENU_COUNT) endIndex = MAIN_MENU_COUNT;

  // Vẽ 3 dòng (LCD dòng 1,2,3)
  for (int i = 0; i < MAIN_ITEMS_PER_PAGE; i++) {
    int menuIndex = startIndex + i;
    if (menuIndex < MAIN_MENU_COUNT) {
      char line[21];
      char arrow = (menuIndex == currentMainIndex) ? '>' : ' ';
      // Ví dụ: "> 1.I2C", " 14.Analog"
      snprintf(line, sizeof(line), "%c%2d.%s", arrow, menuIndex + 1, mainMenuItems[menuIndex]);
      lcdPrintLine(1 + i, line);
    } else {
      // Không có item -> xoá trắng dòng
      lcdPrintLine(1 + i, "");
    }
  }
}

void printI2CSubMenuItem() {
  lcd.clear();
  updateHeaderRow();   // Header: "I2C Menu ..."

  // Cửa sổ 3 dòng menu trên LCD (dòng 1,2,3)
  // Logic cuộn:
  //  - Nếu con trỏ ở 0 hoặc 1 -> hiển thị (0,1,2)
  //  - Nếu con trỏ ở 2 hoặc 3 -> hiển thị (1,2,3)
  int start = 0;
  if (currentI2CIndex <= 1) {
    start = 0;
  } else {
    start = I2C_MENU_COUNT - 3; // với 4 mục -> 1
    if (start < 0) start = 0;   // phòng khi sau này ít hơn 3 mục
  }

  // Vẽ 3 dòng menu tại dòng 1,2,3
  for (int row = 0; row < 3; ++row) {
    int idx = start + row;
    char buf[21];

    if (idx >= 0 && idx < I2C_MENU_COUNT) {
      // Chuỗi dạng: "> 2. Test PCA9685" hoặc "  3. OLED IIC"
      if (idx == currentI2CIndex) {
        snprintf(buf, sizeof(buf), "> %d. %s", idx + 1, i2cSubMenuItems[idx]);
      } else {
        snprintf(buf, sizeof(buf), "  %d. %s", idx + 1, i2cSubMenuItems[idx]);
      }
    } else {
      // Nếu không có mục hợp lệ (trường hợp menu ít hơn 3 mục)
      snprintf(buf, sizeof(buf), " ");
    }

    // Ghi ra LCD ở dòng 1,2,3
    lcdPrintLine(1 + row, buf);
  }
}


#endif
