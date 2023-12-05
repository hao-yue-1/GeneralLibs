//
// Created by yue on 2023/12/5.
//

#ifndef FOC_V1_SC60228_OLED_H
#define FOC_V1_SC60228_OLED_H

void OLED_Init(void);
void OLED_Clear(unsigned char color);
void OLED_PrintPicture(const unsigned char *ptr_pic);
void OLED_PrintStr(unsigned char x, unsigned char y, unsigned char* data, unsigned char text_size);

#endif //FOC_V1_SC60228_OLED_H
