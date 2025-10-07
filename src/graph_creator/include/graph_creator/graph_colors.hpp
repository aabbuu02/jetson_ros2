#pragma once

#include <std_msgs/ColorRGBA.h>

namespace Graph::Colors
{

    struct Color
    {
        float r = 1.0f;
        float g = 1.0f;
        float b = 1.0f;
        float a = 1.0f;

        Color(float red = 1.0f, float green=1.0f, float blue=1.0f, float alpha=1.0f):
            r(red), g(green), b(blue), a(alpha) 
        {}
    };

    const Color RED(1.0, 0.0, 0.0);
    const Color GREEN(0.0, 1.0, 0.0);
    const Color BLUE(0.0, 0.0, 1.0);
    const Color YELLOW(1.0, 1.0, 0.0);
    const Color MAGENTA(1.0, 0.0, 1.0);
    const Color CYAN(0.0, 1.0, 1.0);
    const Color WHITE(1.0, 1.0, 1.0);
    const Color GREY(0.5, 0.5, 0.5);
    const Color BLACK(0.0, 0.0, 0.0);
    
    static inline std_msgs::ColorRGBA getColorRGBAMsg(const Color &color)
    {
        std_msgs::ColorRGBA color_msg;

        color_msg.r = color.r;
        color_msg.g = color.g;
        color_msg.b = color.b;
        color_msg.a = color.a;

        return color_msg;
    }

}