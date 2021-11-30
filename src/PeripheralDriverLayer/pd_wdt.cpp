#include <PeripheralDriverLayer/pd_wdt.h>
#include "iodefine.h"

namespace peripheral_driver {
    void resetWdt() {
        WDT.WDTRR = 0x00;
        WDT.WDTRR = 0xff;
    }

    void initWdt() {
        WDT.WDTCR.BIT.TOPS = 0;
        WDT.WDTCR.BIT.CKS = 1;
        WDT.WDTCR.BIT.RPSS = 3;
        WDT.WDTCR.BIT.RPES = 3;
    }

    void startWdt() {
        WDT.WDTRR = 0x00;
        WDT.WDTRR = 0xff;
    }
}