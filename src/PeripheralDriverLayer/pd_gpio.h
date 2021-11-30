#pragma once

namespace peripheral_driver {
    void initGpio();
    void setDoutP22(bool out);
    void setDoutP21(bool out);
    void setDoutP20(bool out);
    void setDoutPE0(bool out);
    void setDoutPD7(bool out);
    void setDoutPE2(bool out);
    void setDoutPA1(bool out);
    void setDoutPA2(bool out);
}