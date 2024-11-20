/* TDS_Gravity.h - Header file */
#ifndef TDS_GRAVITY_H
#define TDS_GRAVITY_H

class TDS_Gravity {
public:
    TDS_Gravity();
    double readTDSValue(double tdsSensorVoltage);
};

#endif

TDS_Gravity::TDS_Gravity() {}

double TDS_Gravity::readTDSValue(double tdsSensorVoltage) {
    return (133.42 * tdsSensorVoltage * tdsSensorVoltage * tdsSensorVoltage -
            255.86 * tdsSensorVoltage * tdsSensorVoltage +
            857.39 * tdsSensorVoltage) * 0.5;
}

/* Example usage */
// #include "TDS_Gravity.h"
// #include <iostream>
// int main() {
//     TDS_Gravity tds;
//     double voltage = 2.5;  // Example voltage
//     double tdsValue = tds.readTDSValue(voltage);
//     std::cout << "TDS_Gravity Value: " << tdsValue << std::endl;
//     return 0;
// }
