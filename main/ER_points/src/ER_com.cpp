#include <ER/ER_com.h>

extern com::FRAME f;
extern com::USART u1;

void CAMERA_SEND_INFORMATION_ER(output *results)
{
    for(int i = 0; i < 8; i ++)
    {
        if (!isnan(results[i].x) && !isnan(results[i].y) && i != 6 && i != 7)
        {
            f.FRAME_ADD_DATA('?');
            f.FRAME_ADD_DATA(i + 1);
            f.FRAME_ADD_DATA(results[i].x);
            f.FRAME_ADD_DATA(results[i].y);
            f.FRAME_ADD_DATA(results[i].tag);
        }
        if (i == 6 || i == 7)
        {
            f.FRAME_ADD_DATA('?');
            f.FRAME_ADD_DATA(i + 1);
            f.FRAME_ADD_DATA(results[i].x);
            f.FRAME_ADD_DATA(results[i].y);
            f.FRAME_ADD_DATA('B');
        }
    }
    f.FRAME_ADD_DATA('!');
    f.FRAME_SEND_DATA(u1);
}